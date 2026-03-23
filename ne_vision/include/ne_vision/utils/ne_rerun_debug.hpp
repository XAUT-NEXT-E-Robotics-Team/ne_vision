///////////////////////////////////////////////////////////
//                                                       //
//                        .                 .:-:         //
//                       :-:              :-::           //
//                      -----          .:---.            //
//                    .-------.     .:-----:             //
//                   :---------. .:-------.              //
//                  :--------------------.               //
//                ---------------------                  //
//               .-------:. :---------:                  //
//              :-----:.     .-------.                   //
//             .:---:         .-----.                    //
//            .:-:.             :-:                      //
//          .-:.                 .                       //
//         .:                                            //
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

// #ifdef USE_RERUN
// #include <iostream> // Required for std::cerr
// #include "rerun.hpp"
// #include "rerun/demo_utils.hpp"
// #include "rerun/archetypes/scalars.hpp"
// #include "rerun/archetypes/points2d.hpp"
// #include "rerun/recording_stream.hpp"
// #include "rerun/archetypes/scalars.hpp"
// #include "rerun/archetypes/points3d.hpp"
// #include "ne_vision/utils/ne_log.hpp"
// #endif

// namespace ne_vision
// {

// #ifdef USE_RERUN

// class NeRerunDebug
// {
// public:
//   static rerun::RecordingStream& instance()
//   {
//     static NeRerunDebug singleton;
//     return singleton.rec;
//   }

//   NeRerunDebug(const NeRerunDebug&) = delete;
//   NeRerunDebug& operator=(const NeRerunDebug&) = delete;

// private:
//   NeRerunDebug() : rec("ne_vision")
//   {
//     // Capture the connection status
//     auto status = rec.connect_grpc();
//     if (status.is_err())
//     {
//       NV_ERROR("Failed to connect to Rerun server: {}", status.description);
//     }
//   }

//   rerun::RecordingStream rec;
// };

// // Global accessor for convenience
// // Changed to function to avoid static initialization order issues
// inline DummyRecordingStream& nv_rec_g() { return NeRerunDebug::instance(); }

// #define NV_RERUN_CV_IMAGE(cv_mat) nullptr

// #endif // USE_RERUN

// } // namespace ne_vision
