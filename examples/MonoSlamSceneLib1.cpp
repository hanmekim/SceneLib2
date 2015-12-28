/* This file is part of the SceneLib2 Project.
 * http://hanmekim.blogspot.com/2012/10/scenelib2-monoslam-open-source-library.html
 * https://github.com/hanmekim/SceneLib2
 *
 * Copyright (c) 2012 Hanme Kim (hanme.kim@gmail.com)
 *
 * SceneLib2 is an open-source C++ library for SLAM originally designed and
 * implemented by Andrew Davison and colleagues at the University of Oxford.
 *
 * I reimplemented his version with the following objectives;
 *  1. Understand his MonoSLAM algorithm in code level.
 *  2. Replace older libraries (i.e. VW34, GLOW, VNL, Pthread) with newer ones
 *     (Pangolin, Eigen3, Boost).
 *  3. Support USB camera instead of IEEE1394.
 *  4. Make it more portable and convenient by using CMake and git repository.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <pangolin/pangolin.h>

#include "monoslam.h"
#include "support/pangolin_util.h"

using namespace std;

SceneLib2::MonoSLAM *g_monoslam;

int main(int argc, char **argv)
{
  bool  g_next = false;
  bool  g_play = false;
  int   g_frame_id  = 0;

  SceneLib2::Frame  frame;

  // Create & initialise a Scene object
  g_monoslam = new SceneLib2::MonoSLAM();
  g_monoslam->Init("../../data/SceneLib2.cfg");

  // Create main window
  pangolin::CreateWindowAndBind("SceneLib2 - MonoSlamSceneLib1", 800, 480);

#ifndef __APPLE__
  glutInit(&argc, argv);
#endif

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState view_state_3d(pangolin::ProjectionMatrix(g_monoslam->camera_->width_,
                                                                       g_monoslam->camera_->height_,
                                                                       g_monoslam->camera_->fku_,
                                                                       g_monoslam->camera_->fkv_,
                                                                       g_monoslam->camera_->centre_(0),
                                                                       g_monoslam->camera_->centre_(1),
                                                                       0.00001, 1000),
                                            pangolin::ModelViewLookAt(-0.0, 0.18, -1.4, 0.0, 0.13, 0.0, pangolin::AxisY));

  // Add 2 panels, 3D and camera viewers
  pangolin::View  &left_panel1  = pangolin::CreatePanel("left_panel1").SetBounds(0.5, 1.0, 0.0, 0.3);
  pangolin::View  &left_panel2  = pangolin::CreatePanel("left_panel2").SetBounds(0.0, 0.5, 0.0, 0.3);
  pangolin::View  &right_panel1 = pangolin::CreatePanel("right_panel1").SetBounds(0.5, 1.0, 0.3, 0.6);
  pangolin::View  &right_panel2 = pangolin::CreatePanel("right_panel2").SetBounds(0.0, 0.5, 0.3, 0.6);
  pangolin::View  &view_3d      = pangolin::CreateDisplay().SetBounds(0.5, 1.0, 0.6, 1.0, -4./3.)
      .SetHandler(new SceneLib2::Handler3D(view_state_3d, g_monoslam));
  pangolin::View  &view_cam     = pangolin::Display("view_cam").SetBounds(0.0, 0.5, 0.6, 1.0, 4./3.)
      .SetHandler(new SceneLib2::Handler2D(g_monoslam));

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while (!pangolin::ShouldQuit()) {
    if (pangolin::HasResized()) {
      pangolin::DisplayBase().ActivateScissorAndClear();
    }

    // Create GUIs
    static pangolin::Var<string>  cap_graphics_toggles("left_panel1.Graphics Toggles", "");
    static pangolin::Var<bool>    chk_rectify_image_display("left_panel1.Rectify Image Display", false);
    static pangolin::Var<bool>    chk_display_trajectory("left_panel1.Display Trajectory", false);
    static pangolin::Var<bool>    chk_display_3d_features("left_panel1.Display 3D Features", true);
    static pangolin::Var<bool>    chk_display_3d_uncertainties("left_panel1.Display 3D Uncertainties", true);
    static pangolin::Var<bool>    chk_display_2d_descriptors("left_panel1.Display 2D Descriptors", true);
    static pangolin::Var<bool>    chk_display_2d_search_regions("left_panel1.Display 2D Search Regions", true);
    static pangolin::Var<bool>    chk_display_initialisation("left_panel1.Display Initialisation", true);

    static pangolin::Var<string>  cap_main_controls("left_panel2.Main Controls", "");
    static pangolin::Var<bool>    btn_continuous("left_panel2.Continuous", false, false);
    static pangolin::Var<bool>    btn_next("left_panel2.Next", false, false);
    static pangolin::Var<bool>    btn_stop("left_panel2.Stop", false, false);

    static pangolin::Var<string>  cap_controls_toggles("right_panel1.Controls Toggles", "");
    static pangolin::Var<bool>    chk_toggle_tracking("right_panel1.Toggle Tracking", false);
    static pangolin::Var<bool>    chk_enable_mapping("right_panel1.Enable Mapping", true);
    static pangolin::Var<bool>    chk_output_tracked_images("right_panel1.Output Tracked Images", false);
    static pangolin::Var<bool>    chk_output_raw_images("right_panel1.Output Raw Images", false);

    static pangolin::Var<string>  cap_action_controls("right_panel2.Action Controls", "");
    static pangolin::Var<bool>    btn_initialise_manual_feature("right_panel2.Initialise Manual Feature", false, false);
    static pangolin::Var<bool>    btn_initialise_auto_feature("right_panel2.Initialise Auto Feature", false, false);
    static pangolin::Var<bool>    btn_print_robot_state("right_panel2.Print Robot State", false, false);
    static pangolin::Var<bool>    btn_delete_feature("right_panel2.Delete Feature", false, false);
    static pangolin::Var<bool>    btn_save_patch("right_panel2.Save Patch", false, false);
    static pangolin::Var<bool>    btn_quit("right_panel2.Quit", false, false);

    //=========================================================================
    // Activate efficiently by object
    // (3D Handler requires depth testing to be enabled)
    view_3d.ActivateScissorAndClear(view_state_3d);

    g_monoslam->graphic_tool_->Draw3dScene(chk_display_trajectory,
                                           chk_display_3d_features,
                                           chk_display_3d_uncertainties);

    //=========================================================================
    // GET & PROCESS A NEW FRAME & CAMERA VIEW DISPLAY
    view_cam.ActivateScissorAndClear();

    if (g_next) {
      g_monoslam->frame_grabber_->GetFrame(g_frame_id, &frame);

      if (chk_toggle_tracking) {
        g_monoslam->GoOneStep(frame.data,
                              chk_display_trajectory,
                              chk_enable_mapping);
      }

      ++g_frame_id;
    }

    g_monoslam->graphic_tool_->DrawAR(frame.data,
                                      chk_rectify_image_display,
                                      chk_display_trajectory,
                                      chk_display_3d_features,
                                      chk_display_3d_uncertainties,
                                      chk_display_2d_descriptors,
                                      chk_display_2d_search_regions,
                                      chk_display_initialisation);

    // Save Tracked Images
    if(chk_output_tracked_images && g_next) {
      char filename[100];
      sprintf(filename, "composite%04d", g_frame_id);

      view_cam.SaveOnRender(filename);
    }

    // Save Raw Images
    if(chk_output_raw_images && g_next) {
      char filename[100];
      sprintf(filename, "rawoutput%04d.png", g_frame_id);

      cv::imwrite(filename, frame.data);
    }

    //=========================================================================
    // Render our UI panel when we receive input
    if (pangolin::HadInput()) {
      left_panel1.Render();
      left_panel2.Render();
      right_panel1.Render();
      right_panel2.Render();
    }

    g_next = false;

    // Buttons handling
    if (pangolin::Pushed(btn_continuous))
      g_play = true;

    if (pangolin::Pushed(btn_next))
      g_next = true;

    if (pangolin::Pushed(btn_stop))
      g_play = false;

    if (pangolin::Pushed(btn_initialise_manual_feature))
      g_monoslam->InitialiseFeature(frame.data);

    if (pangolin::Pushed(btn_initialise_auto_feature))
      g_monoslam->InitialiseAutoFeature(frame.data);

    if (pangolin::Pushed(btn_print_robot_state))
      g_monoslam->print_robot_state();

    if (pangolin::Pushed(btn_delete_feature))
      g_monoslam->delete_feature();

    if (pangolin::Pushed(btn_save_patch))
      g_monoslam->SavePatch();

    if (pangolin::Pushed(btn_quit))
      exit(0);

    if (g_play)
      g_next = true;

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return  0;
}
