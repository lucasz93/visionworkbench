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

#include <vw/config.h>
#include <vw/Core/Thread.h>
#include <vw/Core/FundamentalTypes.h>
#include <vw/Core/RunOnce.h>

#include <boost/thread.hpp>

#ifdef _PTHREAD_H
#include <dlfcn.h>
#endif

namespace vw {
namespace thread {

  // These static variables are used to generate unique identifiers
  // for threads as identifiers are requested using the static
  // Thread::id() method.
  static vw::uint64 vw_thread_next_available_id = 0;

  // Both the mutex and the thread-local storage need to be available to all
  // destructors (in case they call Thread::id()). If the object being
  // destructed is static and defined in a different file, the destruction
  // order is undefined. That causes a destruction race. To prevent this race,
  // we use the construct-on-first-use idiom. See
  // http://www.parashift.com/c++-faq-lite/ctors.html#faq-10.15 for more info.
  static Mutex& vw_thread_id_mutex() {
    static Mutex* m = new Mutex();
    return *m;
  }

  typedef boost::thread_specific_ptr<vw::uint64> ptr_t;

  static ptr_t& vw_thread_id_ptr() {
    static ptr_t* ptr = new ptr_t();
    return *ptr;
  }

#ifdef _PTHREAD_H
  typedef size_t (*pthread_get_minstack_fn)(pthread_attr_t *attr);
  static pthread_get_minstack_fn __pthread_get_minstack = nullptr;

  vw::RunOnce get_minstack_once = VW_RUNONCE_INIT;

  void get_minstack_init() {
    vw::thread::__pthread_get_minstack = (vw::thread::pthread_get_minstack_fn)dlsym(RTLD_DEFAULT, "__pthread_get_minstack");
  }
#endif

}} // namespace vw::thread

thread_local vw::Thread* vw::Thread::m_self = nullptr;

vw::uint64 vw::Thread::id() {

  // If the thread ID has not been accessed before, we initialize
  // it with a unique ID.
  if (thread::vw_thread_id_ptr().get() == 0) {
    Mutex::Lock lock(thread::vw_thread_id_mutex());
    thread::vw_thread_id_ptr().reset(new vw::uint64(thread::vw_thread_next_available_id++));
  }

  // Then we return the result.
  vw::uint64* result = thread::vw_thread_id_ptr().get();
  return *result;
}

size_t vw::Thread::get_platform_stack_minsize(boost::thread::attributes &attr) {
#ifdef _PTHREAD_H
  // Pthread goof: thread stack size must include space for TLS and other
  // housekeeping. Need to ask glibc how much it wants for that. See
  // https://bugs.launchpad.net/ubuntu/+source/glibc/+bug/1757517
  // https://github.com/rust-lang/rust/issues/6233
  // http://mail.openjdk.java.net/pipermail/core-libs-dev/2019-May/060529.html
  vw::thread::get_minstack_once.run(vw::thread::get_minstack_init);
  return vw::thread::__pthread_get_minstack(attr.native_handle());
#else
  return 0;
#endif
}
