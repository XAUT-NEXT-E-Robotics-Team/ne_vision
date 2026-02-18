///////////////////////////////////////////////////////////
//                                                       //
//                         .                  .:-:       //
//                         :-:               :-::        //
//                       -----           .:---.          //
//                     .-------.      .:-----:           //
//                    :---------. .:-------.             //
//                   :--------------------.              //
//                  ---------------------                //
//                .-------:. :---------:                 //
//               :-----:.      .-------.                 //
//              .:---:          .-----.                  //
//             .:-:.              :-:                    //
//            .-:.                  .                    //
//           .:                                          //
//                                                       //
//    ███╗   ██╗███████╗██╗  ██╗████████╗    ███████╗    //
//    ████╗  ██║██╔════╝╚██╗██╔╝╚══██╔══╝    ██╔════╝    //
//    ██╔██╗ ██║█████╗   ╚███╔╝    ██║       █████╗      //
//    ██║╚██╗██║██╔══╝   ██╔██╗    ██║       ██╔══╝      //
//    ██║ ╚████║███████╗██╔╝ ██╗   ██║       ███████╗    //
//    ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝   ╚═╝       ╚══════╝    //
//                                                       //
///////////////////////////////////////////////////////////
//                                                       //
// Copyright (c) 2026 XAUT NEXT-E. All Rights Reserved.  //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

// Description:
// Rerun singleton wrapper for debugging visualizations.
// To enable, define the macro USE_RERUN during compilation.
// For example, in CMake:
// target_compile_definitions(your_target PRIVATE USE_RERUN)

#pragma once

#ifdef USE_RERUN
#include "rerun.hpp"
#include "rerun/demo_utils.hpp"
#include "rerun/recording_stream.hpp"
#endif

namespace ne_vision
{

#ifdef USE_RERUN

class NeRerunDebug
{
public:
  static rerun::RecordingStream& instance()
  {
    static NeRerunDebug singleton;
    return singleton.rec;
  }

  NeRerunDebug(const NeRerunDebug&) = delete;
  NeRerunDebug& operator=(const NeRerunDebug&) = delete;

private:
  NeRerunDebug() : rec("ne_vision")
  {
    // Spawn a viewer and connect to it.
    // We do NOT exit on failure, because on CI or systems without rerun installed,
    // we don't want to crash.
    rec.spawn();
  }

  rerun::RecordingStream rec;
};

// Global accessor for convenience
// Changed to function to avoid static initialization order issues and eager spawning
inline rerun::RecordingStream& nv_rec_g() { return NeRerunDebug::instance(); }

#else

// When rerun is disabled, provide a dummy implementation that does nothing.
// This avoids needing to include any rerun headers.
class DummyRecordingStream
{
public:
  // Accept any method call with any arguments and do nothing.
  template <typename... Args>
  void log(Args&&...)
  {
  }

  // Accept logging via `<<` and do nothing.
  template <typename T>
  DummyRecordingStream& operator<<(const T&)
  {
    return *this;
  }

  // A dummy result that can be chained for calls like
  // `spawn().value_or_throw()`.
  struct DummyResult
  {
    void exit_on_failure() {}
  };

  DummyResult spawn() { return {}; }
  DummyResult connect(...) { return {}; }
  DummyResult save(...) { return {}; }
};

class NeRerunDebug
{
public:
  static DummyRecordingStream& instance()
  {
    static DummyRecordingStream singleton;
    return singleton;
  }
};

// Global accessor for convenience
// Changed to function to avoid static initialization order issues
inline DummyRecordingStream& nv_rec_g() { return NeRerunDebug::instance(); }

#endif // USE_RERUN

} // namespace ne_vision
