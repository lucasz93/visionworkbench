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


/// \file Core/Thread.h
///
/// A very simple, abstracted threading system.  Currently this is
/// simply built on top of Boost threads, but its abstracted so that
/// it's easy to port the Vision Workbench to environments where Boost
/// threads do not work or otherwise cannot be used, and because the
/// Boost threads design itself is expected to be overhauled at some
/// point.
///
/// To create a thread, pass in any object that has the operator()
/// defined.
///
/// The rest of the interface is straightforward.  These are
/// the things you will need to implement if you want to use a
/// different thread library.  The interface consists of:
///
/// * A Thread class, whose constructor takes a shared pointer to a
///   Task.  The constructor should invoke the Task's operator()
///   function in a new thread, and the destructor should join the
///   child thread.  The static yield() function should yield the
///   active thread, and the static sleep_ms() function should sleep
///   the active thread for the given number of milliseconds.  The
///   Thread class may be non-copyable.
///
/// * A Mutex class, implementing a simple mutual exclusion lock.
///   This should have no methods other than the default constructor
///   and destructor.  All locking and unlocking takes place through
///   a nested Mutex::Lock class, whose constructor takes a Mutex&
///   to lock and whose destructor unlocks it.  Both the Mutex and
///   Lock classes may be non-copyable.
///

#ifndef __VW_CORE_THREAD_H__
#define __VW_CORE_THREAD_H__

#include <vw/Core/FundamentalTypes.h>

#include <boost/thread/locks.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/version.hpp>

#include <unordered_map>

namespace vw {

  // --------------------------------------------------------------
  //                            MUTEX
  // --------------------------------------------------------------

  /// A wrapper around some boost::mutex classes.
  /// - See docs: http://www.boost.org/doc/libs/1_59_0/doc/html/thread/synchronization.html
  /// - There are three locks being managed here:
  ///   - lock         = Normal, exclusive access lock.
  ///   - lock_shared  = Non-exclusive lock.
  ///   - lock_upgrade = Similar to lock_shared, but with the ability to upgrade to lock.
  ///                    Having this intermediate lock step is necessary to secure exclusive 
  ///                    access (lock) in a timely manner.
  class Mutex : private boost::shared_mutex {

    friend class WriteLock;
    friend class ReadLock;

  public:
    inline Mutex() {}

    /// Block until you own the requested lock.
    void lock        () { boost::shared_mutex::lock();         }
    void lock_shared () { boost::shared_mutex::lock_shared();  }
    void lock_upgrade() { boost::shared_mutex::lock_upgrade(); }
    
    /// Non-blocking attempt to obtain ownership of a lock.
    bool try_lock        () { return boost::shared_mutex::try_lock();         }
    bool try_lock_shared () { return boost::shared_mutex::try_lock_shared();  }
    bool try_lock_upgrade() { return boost::shared_mutex::try_lock_upgrade(); }
    
    /// Release ownership of a lock.
    void unlock        () { boost::shared_mutex::unlock();         }
    void unlock_shared () { boost::shared_mutex::unlock_shared();  }
    void unlock_upgrade() { boost::shared_mutex::unlock_upgrade(); }
    
    /// Atomic operations to switch ownership from one type of lock to another.
    void unlock_upgrade_and_lock       () { boost::shared_mutex::unlock_upgrade_and_lock(); }
    void unlock_and_lock_upgrade       () { boost::shared_mutex::unlock_and_lock_upgrade(); }
    void unlock_upgrade_and_lock_shared() { boost::shared_mutex::unlock_upgrade_and_lock_shared(); }

    /// A unique scoped lock class, used to lock and unlock a Mutex (only one can own at a time).
    /// - Automatically locks the mutex when constructed, unlocks on destruction.
    class WriteLock : private boost::unique_lock<Mutex>, private boost::noncopyable {
    public:
      inline WriteLock( Mutex &mutex ) : boost::unique_lock<Mutex>( mutex ) {}
      void lock()          { boost::unique_lock<Mutex>::lock(); }
      void unlock()        { boost::unique_lock<Mutex>::unlock(); }
    };
    
    /// A scoped lock class, used to lock and unlock a Mutex (allows shared ownership).
    /// - Automatically locks the mutex when constructed, unlocks on destruction.
    class ReadLock : private boost::shared_lock<Mutex>, private boost::noncopyable {
    public:
      inline ReadLock( Mutex &mutex ) : boost::shared_lock<Mutex>( mutex ) {}
      void lock()          { boost::shared_lock<Mutex>::lock(); }
      void unlock()        { boost::shared_lock<Mutex>::unlock(); }
    };

    typedef class WriteLock Lock; /// By default, use the non-shared lock type.
  };// End class Mutex

  /// A wrapper around the boost::recursive_mutex class.
  class RecursiveMutex : private boost::recursive_mutex {

    friend class Lock;

  public:
    inline RecursiveMutex() {}

    void lock()   { boost::recursive_mutex::lock(); }
    void unlock() { boost::recursive_mutex::unlock(); }

    // A unique scoped lock class, used to lock and unlock a Mutex (only one can own at a time).
    class Lock : private boost::unique_lock<RecursiveMutex>, private boost::noncopyable {
    public:
      inline Lock( RecursiveMutex &mutex ) : boost::unique_lock<RecursiveMutex>( mutex ) {}
      void lock()   { boost::unique_lock<RecursiveMutex>::lock(); }
      void unlock() { boost::unique_lock<RecursiveMutex>::unlock(); }
    }; // End class Lock
    
  }; // End class RecursiveLock


  // --------------------------------------------------------------
  //                            THREAD
  // --------------------------------------------------------------

  /// Run at the end of thread execution.
  class ThreadEventListener {
  public:
    virtual ~ThreadEventListener() {}

    virtual void finish(uint64_t id) {}
  };

  /// A thread class, that runs a "Task", which is an object or
  /// function that has the operator() defined.  When the Thread object
  /// is destroyed it will join the child thread if it has not already terminated.
  class Thread : private boost::noncopyable {
    friend class TaskHelper;

    // Pointer to the active thread object. Will be null for the main thread.
    // Allows objects to to listen for thread destruction.
    static thread_local Thread* m_self;

    std::vector<boost::shared_ptr<ThreadEventListener>> m_listeners;

    boost::thread m_thread;

    // For some reason, the boost thread library makes a copy of the
    // Task object before handing it off to the thread.  This is
    // annoying, because it means that the parent thread no longer has
    // direct access to the child thread's instance of the task object.
    // This helper allows the parent thread to retain direct access to
    // the child instance of the task.
    template <class TaskT>
    class TaskHelper {
      boost::shared_ptr<TaskT> m_task;
      Thread* m_thread;
    public:
      TaskHelper(Thread *thread, boost::shared_ptr<TaskT> task) : m_thread(thread), m_task(task) {}
      TaskHelper(Thread *thread, const TaskT &task) : m_thread(thread), m_task(new TaskT(task)) {}
      void operator() () { 
        Thread::m_self = m_thread;

        (*m_task)();

        auto thread_id = Thread::id();
        for (auto &t : m_thread->m_listeners)
          t->finish(thread_id);

        Thread::m_self = nullptr;
      }
    };

  public:

    /// This variant of the constructor takes a Task that is copy
    /// constructable.  The thread made a copy of the task, and this
    /// instance is no longer directly accessibly from the parent thread.
    template<class TaskT>
    inline Thread( TaskT task ) {
      boost::thread::attributes attr;

      // Ames Stereo Pipeline was running out of stack space when running
      // stereo_tri in threads. Upping stack size from default 8KB to 32KB.
      attr.set_stack_size(8388608 + get_platform_stack_minsize(attr));

      m_thread = std::move(boost::thread(attr, TaskHelper<TaskT>(this, task)));
    }

    /// This variant of the constructor takes a shared pointer to a task.
    /// The thread makes a copy of the shared pointer task, allowing
    /// the parent to still access the task instance that is running in the thread.
    template<class TaskT>
    inline Thread( boost::shared_ptr<TaskT> task ) {
      boost::thread::attributes attr;

      // Ames Stereo Pipeline was running out of stack space when running
      // stereo_tri in threads. Upping stack size from default 8KB to 32KB.
      attr.set_stack_size(8388608 + get_platform_stack_minsize(attr));

      m_thread = std::move(boost::thread(attr, TaskHelper<TaskT>(this, task)));
    }

    /// Destroys the thread. User is expected to either call join() themselves,
    /// or let the thread run free. We don't call join() here because most users
    /// will call join(), and a second call to join() is undefined.
    inline ~Thread() { }

    /// The current thread of execution blocks until this thread
    /// finishes execution of the task and all resources are reclaimed.
    inline void join() { m_thread.join(); }

    static inline Thread* self() { return m_self; }
    
    void add_listener(boost::shared_ptr<ThreadEventListener> listener) {
      m_listeners.push_back(listener);
    }

    // --------------
    // STATIC METHODS
    // --------------

    /// Returns a unique ID for the current thread.  The ID for a
    /// thread is not determined until the thread calls the id()
    /// function for the first time, so there is no guarantee that IDs
    /// will be assigned in the same order that threads are created.
    static vw::uint64 id();

    /// Cause the current thread to yield the remainder of its
    /// execution time to the kernel's scheduler.
    static inline void yield() { boost::thread::yield(); }

    /// Cause the current thread to sleep for a specified amount of
    /// time.  The thread will not be scheduled to run at all for the
    /// duration, so machine resources are free for other
    /// threads/processes.
    static inline void sleep_ms( uint32 milliseconds ) {
      boost::xtime xt;
#if BOOST_VERSION >= 105000
      boost::xtime_get(&xt, boost::TIME_UTC_);
#else
      boost::xtime_get(&xt, boost::TIME_UTC);
#endif
      while (milliseconds >= 1000) {
        xt.sec++;
        milliseconds -= 1000;
      }
      xt.nsec += static_cast<uint32>(1e6) * milliseconds;
      boost::thread::sleep(xt);
    }

    /// Gets the size of the stack desired by the platform.
    /// Filled by things like thread-local storage.
    static size_t get_platform_stack_minsize(boost::thread::attributes &attr);
  }; // End class Thread

  // Adapted from https://github.com/AlexeyAB/object_threadsafe
  // boost::shared_lock (on Linux) has worse performance than a standard lock.
  // This is due to the overhead in scheduling -> preventing starvation of exclusive locks.
  //
  // This contention free lock performs best in places with an overwhelming number
  // of reads, and very few writes.
  //
  // Oh yeah, and it can't be upgraded from a shared to exclusive lock.
  //
  // The original version had a template parameter for LUT size. For runtime flexibility,
  // this has been replaced with the double the thread pool size.
  class FastSharedMutex {
		std::atomic < bool > want_x_lock;
		//struct cont_free_flag_t { alignas(std::hardware_destructive_interference_size) std::atomic<int> value; cont_free_flag_t() { value = 0; } }; // C++17
		struct cont_free_flag_t {
		  char tmp[60];
		  std::atomic < int > value;
		  cont_free_flag_t() {
		    value = 0;
		  }
		}; // tmp[] to avoid false sharing

    typedef std::vector < cont_free_flag_t > array_slock_t;

		const std::shared_ptr< array_slock_t > shared_locks_array_ptr; // 0 - unregistred, 1 registred & free, 2... - busy
		char avoid_falsesharing_1[64];

		array_slock_t & shared_locks_array;
		char avoid_falsesharing_2[64];

		int recursive_xlock_count;

		enum index_op_t {
		  unregister_thread_op,
		  get_index_op,
		  register_thread_op
		};

    typedef uint64_t thread_id_t;
    std::atomic < thread_id_t > owner_thread_id;
		uint64_t get_fast_this_thread_id() {
		  return vw::Thread::id();
		}

#if(_WIN32 && _MSC_VER < 1900) // only for MSVS 2013
		std::array < int64_t, contention_free_count > register_thread_array;

		int get_or_set_index(index_op_t index_op = get_index_op, int set_index = -1) {
		  if (index_op == get_index_op) { // get index
		    auto
		    const thread_id = get_fast_this_thread_id();

		    for (size_t i = 0; i < register_thread_array.size(); ++i) {
		      if (register_thread_array[i] == thread_id) {
		        set_index = i; // thread already registred                
		        break;
		      }
		    }
		  } else if (index_op == register_thread_op) { // register thread
		    register_thread_array[set_index] = get_fast_this_thread_id();
		  }
		  return set_index;
		}

#else
		struct unregister_t {
		  int thread_index;
		  std::shared_ptr < array_slock_t > array_slock_ptr;
		  unregister_t(int index, std::shared_ptr < array_slock_t >
		    const & ptr): thread_index(index), array_slock_ptr(ptr) {}
		  unregister_t(unregister_t && src): thread_index(src.thread_index), array_slock_ptr(std::move(src.array_slock_ptr)) {}~unregister_t() {
		    if (array_slock_ptr.use_count() > 0)( * array_slock_ptr)[thread_index].value--;
		  }
		};

		int get_or_set_index(index_op_t index_op = get_index_op, int set_index = -1) {
		  thread_local static std::unordered_map < void * , unregister_t > thread_local_index_hashmap;
		  // get thread index - in any cases
		  auto it = thread_local_index_hashmap.find(this);
		  if (it != thread_local_index_hashmap.cend())
		    set_index = it -> second.thread_index;

		  if (index_op == unregister_thread_op) { // unregister thread
		    if (shared_locks_array[set_index].value == 1) // if isn't shared_lock now
		      thread_local_index_hashmap.erase(this);
		    else
		      return -1;
		  } else if (index_op == register_thread_op) { // register thread
		    thread_local_index_hashmap.emplace(this, unregister_t(set_index, shared_locks_array_ptr));

		    // remove info about deleted contfree-mutexes
		    for (auto it = thread_local_index_hashmap.begin(), ite = thread_local_index_hashmap.end(); it != ite;) {
		      if (it -> second.array_slock_ptr -> at(it -> second.thread_index).value < 0) // if contfree-mtx was deleted
		        it = thread_local_index_hashmap.erase(it);
		      else
		        ++it;
		    }
		  }
		  return set_index;
		}

#endif

		public:
		  FastSharedMutex(int contention_free_count = -1);
		  ~FastSharedMutex() {
		    for (auto & i: shared_locks_array) i.value = -1;
		  }

		bool unregister_thread() {
		  return get_or_set_index(unregister_thread_op) >= 0;
		}

		int register_thread() {
		  int cur_index = get_or_set_index();

		  if (cur_index == -1) {
		    if (shared_locks_array_ptr.use_count() <= (int) shared_locks_array.size()) // try once to register thread
		    {
		      for (size_t i = 0; i < shared_locks_array.size(); ++i) {
		        int unregistred_value = 0;
		        if (shared_locks_array[i].value == 0)
		          if (shared_locks_array[i].value.compare_exchange_strong(unregistred_value, 1)) {
		            cur_index = i;
		            get_or_set_index(register_thread_op, cur_index); // thread registred success
		            break;
		          }
		      }
		      //std::cout << "\n thread_id = " << std::this_thread::get_id() << ", register_thread_index = " << cur_index <<
		      //    ", shared_locks_array[cur_index].value = " << shared_locks_array[cur_index].value << std::endl;
		    }
		  }
		  return cur_index;
		}

		void lock_shared() {
		  int
		  const register_index = register_thread();

		  if (register_index >= 0) {
		    int recursion_depth = shared_locks_array[register_index].value.load(std::memory_order_acquire);
		    assert(recursion_depth >= 1);

		    if (recursion_depth > 1)
		      shared_locks_array[register_index].value.store(recursion_depth + 1, std::memory_order_release); // if recursive -> release
		    else {
		      shared_locks_array[register_index].value.store(recursion_depth + 1, std::memory_order_seq_cst); // if first -> sequential
		      while (want_x_lock.load(std::memory_order_seq_cst)) {
		        shared_locks_array[register_index].value.store(recursion_depth, std::memory_order_seq_cst);
		        for (volatile size_t i = 0; want_x_lock.load(std::memory_order_seq_cst); ++i)
		          if (i % 100000 == 0) vw::Thread::yield();
		        shared_locks_array[register_index].value.store(recursion_depth + 1, std::memory_order_seq_cst);
		      }
		    }
		    // (shared_locks_array[register_index] == 2 && want_x_lock == false) ||     // first shared lock
		    // (shared_locks_array[register_index] > 2)                                 // recursive shared lock
		  } else {
		    if (owner_thread_id.load(std::memory_order_acquire) != get_fast_this_thread_id()) {
		      size_t i = 0;
		      for (bool flag = false; !want_x_lock.compare_exchange_weak(flag, true, std::memory_order_seq_cst); flag = false)
		        if (++i % 100000 == 0) vw::Thread::yield();
		      owner_thread_id.store(get_fast_this_thread_id(), std::memory_order_release);
		    }
		    ++recursive_xlock_count;
		  }
		}

		void unlock_shared() {
		  int
		  const register_index = get_or_set_index();

		  if (register_index >= 0) {
		    int
		    const recursion_depth = shared_locks_array[register_index].value.load(std::memory_order_acquire);
		    assert(recursion_depth > 1);

		    shared_locks_array[register_index].value.store(recursion_depth - 1, std::memory_order_release);
		  } else {
		    if (--recursive_xlock_count == 0) {
		      owner_thread_id.store(decltype(owner_thread_id)(), std::memory_order_release);
		      want_x_lock.store(false, std::memory_order_release);
		    }
		  }
		}

		void lock() {
		  // forbidden upgrade S-lock to X-lock - this is an excellent opportunity to get deadlock
		  int
		  const register_index = get_or_set_index();
		  if (register_index >= 0)
		    assert(shared_locks_array[register_index].value.load(std::memory_order_acquire) == 1);

		  if (owner_thread_id.load(std::memory_order_acquire) != get_fast_this_thread_id()) {
		    size_t i = 0;
		    for (bool flag = false; !want_x_lock.compare_exchange_weak(flag, true, std::memory_order_seq_cst); flag = false)
		      if (++i % 1000000 == 0) vw::Thread::yield();

		    owner_thread_id.store(get_fast_this_thread_id(), std::memory_order_release);

		    for (auto & i: shared_locks_array)
		      while (i.value.load(std::memory_order_seq_cst) > 1);
		  }

		  ++recursive_xlock_count;
		}

		void unlock() {
		  assert(recursive_xlock_count > 0);
		  if (--recursive_xlock_count == 0) {
		    owner_thread_id.store(decltype(owner_thread_id)(), std::memory_order_release);
		    want_x_lock.store(false, std::memory_order_release);
		  }
		}

    /// A unique scoped lock class, used to lock and unlock a Mutex (only one can own at a time).
    /// - Automatically locks the mutex when constructed, unlocks on destruction.
    class WriteLock {
      FastSharedMutex & ref_mtx;
    public:
      WriteLock(FastSharedMutex & mtx): ref_mtx(mtx) {
        ref_mtx.lock();
      }
      ~WriteLock() {
        ref_mtx.unlock();
      }
    };

    /// A scoped lock class, used to lock and unlock a Mutex (allows shared ownership).
    /// - Automatically locks the mutex when constructed, unlocks on destruction.
    class ReadLock {
      FastSharedMutex & ref_mtx;
    public:
      ReadLock(FastSharedMutex & mtx): ref_mtx(mtx) {
        ref_mtx.lock_shared();
      }
      ~ReadLock() {
        ref_mtx.unlock_shared();
      }
    };
  }; // End class FastSharedMutex

} // namespace vw

#endif // __VW_CORE_THREAD_H__
