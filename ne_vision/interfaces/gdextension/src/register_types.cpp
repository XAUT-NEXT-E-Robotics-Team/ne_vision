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
// This file is the main entry point for the GDExtension module. It includes the
// necessary headers and defines the initialization and uninitialization
// functions for the module. The initialize_ne_vision_gd_module function is
// responsible for registering all the classes that will be exposed to Godot,
// while the uninitialize_ne_vision_gd_module function can be used to perform
// any necessary cleanup when the module is unloaded.

#include "gdextension/register_types.h"

#include <gdextension_interface.h>

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "gdextension/ne_vision_gd.hpp"

using namespace godot;

void initialize_ne_vision_gd_module(ModuleInitializationLevel p_level)
{
  if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE)
  {
    return;
  }

  GDREGISTER_CLASS(ne_vision::gdextension::NeVisionGd)
}

void uninitialize_ne_vision_gd_module(ModuleInitializationLevel p_level)
{
  if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE)
  {
    return;
  }
}

extern "C"
{
  // Initialization.
  GDExtensionBool GDE_EXPORT ne_vision_gd_library_init(
      GDExtensionInterfaceGetProcAddress p_get_proc_address,
      GDExtensionClassLibraryPtr         p_library,
      GDExtensionInitialization*         r_initialization)
  {
    godot::GDExtensionBinding::InitObject init_obj(
        p_get_proc_address, p_library, r_initialization);

    init_obj.register_initializer(initialize_ne_vision_gd_module);
    init_obj.register_terminator(uninitialize_ne_vision_gd_module);
    init_obj.set_minimum_library_initialization_level(
        MODULE_INITIALIZATION_LEVEL_SCENE);

    return init_obj.init();
  }
}
