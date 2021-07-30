// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


/// \file CameraModel.h
///
/// This file contains the abstract base class from which all camera
/// models must derive their interface.
///
#ifndef __VW_CAMERA_CAMERAMODEL_H__
#define __VW_CAMERA_CAMERAMODEL_H__

#include <fstream>
#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/Core/Thread.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>

#include <boost/smart_ptr/shared_ptr.hpp>

namespace vw {
namespace camera {

  /// This is the abstract base class for a camera model object.  You
  /// should create a child that adheres to the interface set forth
  /// below.  If your camera model subclass does not implement one of
  /// these methods, it should vw_throw a vw::NotImplErr() exception.
  ///
  /// The most fundamental camera model operation is the forward
  /// projection method ( point_to_pixel() ) that "images" a point in 3D
  /// space; this returns the pixel location where the projection
  /// intersects the image plane.
  ///
  /// It is often necessary to perform the inverse operation:
  /// projecting a ray from the camera center through the pixel in an
  /// image plane to determine the direction of the original 3D point.
  /// The 3D point corresponding to pixel (u,v) lies along a ray whose
  /// origin can be determined by calling camera_center(Vector2(u,v))
  /// and whose direction can be determined by calling
  /// pixel_to_vector(Vector2(u,v)).  Note that the absolute position
  /// of the 3D point cannot be computed from the pixel location in a
  /// single image; this information is lost when the point is forward
  /// projected.  However stereo vision techniques can be used to
  /// determine the location of the original point by intersecting two
  /// rays from two distinct cameras.
  class CameraModel {

  public:

    virtual ~CameraModel() {}

    //------------------------------------------------------------------
    // Generic Camera Model Interface
    //------------------------------------------------------------------

    /// Computes the image of the point 'point' in 3D space on the
    /// image plane.  Returns a pixel location (col, row) where the
    /// point appears in the image.  It is possible that the selected
    /// point will not be imaged by the camera (e.g. if it lies behind
    /// the camera).  In this case the method should vw_throw a
    /// vw::camera::PointToPixelErr()
    virtual Vector2 point_to_pixel (Vector3 const& point) const = 0;

    /// Returns a pointing vector from the camera center through the
    /// position of the pixel 'pix' on the image plane.  For
    /// consistency, the pointing vector should generally be normalized.
    virtual Vector3 pixel_to_vector(Vector2 const& pix) const = 0;

    /// Returns the camera center in the frame of reference of the
    /// camera model.  This method is often used to find the origin of
    /// a ray emanating from the focal point of the camera through a
    /// pixel on the image plane (e.g. for computing ray-ray
    /// intersection in a stereo vision algorithm).
    /// - Generally the input pixel is only used for linescan cameras.
    virtual Vector3 camera_center(Vector2 const& pix) const = 0;

    /// Subclasses must define a method that return the camera type as a string.
    virtual std::string type() const = 0;

    /// Returns the pose (as a quaternion) of the camera for a given
    /// pixel. It represents the rotation from the camera frame to world frame.
    /// - Generally the input pixel is only used for linescane cameras.
    virtual Quaternion<double> camera_pose(Vector2 const& /*pix*/) const;

    /// Has this camera been tweaked?
    virtual bool is_adjusted() const { return false; }

    /// Function to "map" the CAHVORE parameters into CAHV
    virtual boost::shared_ptr<CameraModel> linearize_camera(Vector2i const& cahvore_image_size, Vector2i const& cahv_image_size) const {
      throw std::runtime_error("linearize_camera for this CameraModel");
    }

    // This should be a value which can never occur in normal
    // circumstances, but it most not be made up of NaN values, as those
    // are hard to compare.
    inline static Vector2 invalid_pixel(){ return Vector2(-1e8, -1e8); }
  };
  typedef boost::shared_ptr<CameraModel> CameraModelPtr;

  /// This class is useful if you have an existing camera model, and
  /// you want to systematically "tweak" its extrinsic and intrinsic
  /// parameters. This is particularly useful in Bundle
  /// Adjustment. The tweaks will be camera rotation around a fixed
  /// center, camera translation, and in pixel space, pixel offset
  /// and pixel scale.

  // The adjusted camera is obtained by applying to the unadjusted
  // camera the rigid transform:
  //   m_rotation*( P - m_rotation_center ) + m_rotation_center + m_translation
  // Here, P is in the world coordinate system. P is a point on the
  // unadjusted camera, and it becomes after the transform a point on
  // the adjusted camera.
  class AdjustedCameraModel : public CameraModel {

    boost::shared_ptr<CameraModel> m_camera;
    Vector3 m_translation;
    Quat m_rotation;
    Quat m_rotation_inverse;

    // apply the rotations in respect to this point.
    Vector3 m_rotation_center;

    // This offset is used when the images are cropped.
    // Adding the offset to a pixel converts from the cropped
    // image pixels to original camera pixels.
    Vector2 m_pixel_offset;

    // The scale is used when we want to use resampled images.
    // Multiplying a pixel in the scaled image by the scale
    // and then adding the pixel offset converts to the
    // original camera pixels.
    double m_scale;


  public:
    AdjustedCameraModel(boost::shared_ptr<CameraModel> camera_model,
                        Vector3 const& translation  = Vector3(),
                        Quat    const& rotation     = Quat(math::identity_matrix<3>()),
                        Vector2 const& pixel_offset = Vector2(),
                        double scale = 1.0);

    virtual ~AdjustedCameraModel();
    virtual std::string type() const;

    Vector3 translation () const;
    Quat    rotation    () const;
    Vector2 pixel_offset() const;
    double  scale       () const;
    Matrix<double,3,3> rotation_matrix() const;

    Vector3 axis_angle_rotation() const;
    void    set_rotation(Quat const&);

    template <class MatrixT>
    void set_rotation(MatrixBase<MatrixT> const& m) {
      m_rotation = Quat(m.impl());
      m_rotation_inverse = inverse(m_rotation);
    }
    template <class VectorT>
    void set_translation(VectorBase<VectorT> const& v) {
      m_translation = v.impl();
    }

    template <class VectorT>
    void set_axis_angle_rotation(VectorBase<VectorT> const& v) {
      this->set_rotation( axis_angle_to_quaternion(v.impl()) );
    }
    template <class VectorT>
    void set_pixel_offset(VectorBase<VectorT> const& v) {
      m_pixel_offset = v.impl();
    }

    void set_scale(double scale){
      m_scale = scale;
    }

    virtual Vector2 point_to_pixel (Vector3 const&) const;
    virtual Vector3 pixel_to_vector(Vector2 const&) const;
    virtual Vector3 camera_center  (Vector2 const&) const;
    virtual Quat    camera_pose    (Vector2 const&) const;

    Vector3 adjusted_point(Vector3 const& point) const;
    
    boost::shared_ptr<CameraModel> unadjusted_model(){
      return m_camera;
    }

    boost::shared_ptr<CameraModel> unadjusted_model() const{
      return m_camera;
    }

    /// Has this camera been tweaked?
    virtual bool is_adjusted() const { return true; }

    /// Modify the adjustments by applying on top of them a rotation + translation
    /// transform with the origin at the center of the planet (such as
    /// output by pc_align's forward or inverse computed alignment transform).
    void apply_transform(vw::Matrix4x4 const& M);
    
    void write(std::string const&);
    void read (std::string const&);

    friend std::ostream& operator<<(std::ostream&, AdjustedCameraModel const&);
  };

  std::ostream& operator<<(std::ostream&, AdjustedCameraModel const&);

  //-------------------------------------------------------------------------------------
  // Allocators

  /// Interface.
  class CameraModelAllocator {
  public:
    virtual boost::shared_ptr<CameraModel> allocate() = 0;
    virtual void                           deallocate(CameraModel *cam) = 0;
  };
  typedef boost::shared_ptr<CameraModelAllocator> CameraModelAllocatorPtr;

  /// Use with thread safe CameraModels.
  class CameraModelNoAllocator : public CameraModelAllocator {
  public:
    static CameraModelAllocatorPtr create(CameraModelPtr model) {
      return CameraModelAllocatorPtr(new CameraModelNoAllocator(model));
    }

    CameraModelPtr allocate() final override { return m_model; }
    void deallocate(CameraModel *cam) final override {}

  private:
    CameraModelNoAllocator() = delete;
    CameraModelNoAllocator(const CameraModelNoAllocator& rhs) = delete;
    
    CameraModelNoAllocator(CameraModelPtr model) : m_model(model) {}

    CameraModelPtr m_model;
  };

  /// Keeps a cache of previously allocated cameras.
  /// Can be used for CameraModels which aren't thread safe, like ISIS cameras.
  class CameraModelLruAllocator : public CameraModelAllocator, public std::enable_shared_from_this<CameraModelLruAllocator> {
  public:
    static CameraModelAllocatorPtr create(std::function<CameraModel*()> create, std::function<void(CameraModel*)> destroy = delete_camera_model, int max_size = 0) {
      return CameraModelAllocatorPtr(new CameraModelLruAllocator(create, destroy, max_size));
    }

    ~CameraModelLruAllocator() {
      {
        vw::Mutex::WriteLock lock(m_mutex);
        if (m_lru.size() != m_allocated)
          VW_OUT(ErrorMessage, "CameraModelLruAllocator") << "Destroying an LRU allocator while some of its models are alive!" << std::endl;
      }

      while (m_lru.size()) {
        m_destroy(m_lru.front());
        m_lru.pop_front();
      }
    }

    boost::shared_ptr<CameraModel> allocate() final override {
      vw::Mutex::WriteLock lock(m_mutex);

      if (m_allocated == m_max_size && !m_lru.size())
        throw std::runtime_error("CameraModelLruAllocator ran out of cached instances.");

      CameraModel* instance = nullptr;

      if (m_lru.size()) {
        instance = m_lru.front();
        m_lru.pop_front();
      } else {
        instance = m_create();
        m_allocated++;
      }

      // 'shared_from_this' only works when the object is already defined as a shared_ptr.
      // CameraModelLruAllocator::create only creates as shared_ptrs, so we're safe.
      auto self = shared_from_this();
      auto deleter = [self](CameraModel* cam) {
        self->deallocate(cam);
      };

      return boost::shared_ptr<CameraModel>(instance, deleter);
    }

    void deallocate(CameraModel *cam) final override {
      vw::Mutex::WriteLock lock(m_mutex);
      m_lru.push_back(cam);
    }

  private:
    CameraModelLruAllocator() = delete;
    CameraModelLruAllocator(const CameraModelLruAllocator& rhs) = delete;

    CameraModelLruAllocator(std::function<CameraModel*()> create, std::function<void(CameraModel*)> destroy, int max_size) 
      : m_create(create)
      , m_destroy(destroy)
      , m_allocated(0)
      , m_max_size(max_size > 0 ? max_size : std::numeric_limits<int>::max())
    {
    }

    static void delete_camera_model(CameraModel *cam) {
      delete cam;
    }

    std::deque<CameraModel*>          m_lru;
    int                               m_allocated;
    const int                         m_max_size;
    vw::Mutex                         m_mutex;

    std::function<CameraModel*()>     m_create;
    std::function<void(CameraModel*)> m_destroy;
  };

  //-------------------------------------------------------------------------------------
  // Helper functions

  // If this is an adjusted model, get the unadjusted one.
        CameraModel* unadjusted_model(      CameraModel * cam);
  const CameraModel* unadjusted_model(const CameraModel * cam);

  /// Error during projection of a 3D point onto the image plane.
  VW_DEFINE_EXCEPTION(PointToPixelErr, vw::Exception);

  /// Error during reverse projection of a pixel to a pointing vector
  /// from the camera center.
  VW_DEFINE_EXCEPTION(PixelToRayErr, vw::Exception);

  /// Given a point in world coordinates, convert it to camera coordinates.
  /// - This is generic coordinate frame code but there is nowhere else to put it.
  inline Vector3 point_to_camera_coord(Vector3 const& camera_position,
                                       Quat    const& camera_pose,     
                                       Vector3 const& point) {
    return inverse(camera_pose).rotate(point - camera_position);
  }


  //========================================================================================
  // Ray correction functions

  // WARNING: These currently only work for Earth!

  /// Returns the velocity corrected to account for the planetary rotation.
  /// - For efficiency, requires the uncorrected look vector at this location.
  Vector3 get_rotation_corrected_velocity(Vector3 const& camera_center,
                                          Vector3 const& camera_velocity,
                                          double         mean_earth_radius,
                                          Vector3 const& uncorrected_vector);

  /// Adjust a pixel vector to account for velocity aberration.
  Vector3 apply_velocity_aberration_correction(Vector3 const& camera_center,
                                               Vector3 const& camera_velocity,
                                               double         mean_earth_radius,
                                               Vector3 const& uncorrected_vector);

  /// Simple atmospheric atmospheric correction method.
  double saastamoinen_atmosphere_correction(double camera_alt, double ground_alt, double alpha);

  /// Account for atmospheric refraction.
  Vector3 apply_atmospheric_refraction_correction(Vector3 const& camera_center,
                                                  double         mean_earth_radius,
                                                  double         mean_surface_elevation,
                                                  Vector3 const& uncorrected_vector);

}} // namespace vw::camera

#endif // __VW_CAMERA_CAMERAMODEL_H__
