#include <pangolin/display/display.h>
#include <pangolin/pangolin.h>

void setImageData(unsigned char * imageArray, int size){
  for(int i = 0 ; i < size;i++) {
    imageArray[i] = (unsigned char)(rand()/(RAND_MAX/255.0));
  }
}

void testMultiTuto() {
    // Create OpenGL window in single line
  pangolin::CreateWindowAndBind("Main",640,480);
  
  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);  

  // Issue specific OpenGl we might need
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000);
  pangolin::OpenGlRenderState s_cam(proj, pangolin::ModelViewLookAt(1,0.5,-2,0,0,0, pangolin::AxisY) );
  pangolin::OpenGlRenderState s_cam2(proj, pangolin::ModelViewLookAt(0,0,-2,0,0,0, pangolin::AxisY) );

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam1 = pangolin::Display("cam1")
    .SetAspect(640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::View& d_cam2 = pangolin::Display("cam2")
    .SetAspect(640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(s_cam2));

  pangolin::View& d_cam3 = pangolin::Display("cam3")
    .SetAspect(640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::View& d_cam4 = pangolin::Display("cam4")
    .SetAspect(640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(s_cam2));

  pangolin::View& d_img1 = pangolin::Display("img1")
    .SetAspect(640.0f/480.0f);

  pangolin::View& d_img2 = pangolin::Display("img2")
    .SetAspect(640.0f/480.0f);

  // LayoutEqual is an EXPERIMENTAL feature - it requires that all sub-displays
  // share the same aspect ratio, placing them in a raster fasion in the
  // viewport so as to maximise display size.
  pangolin::Display("multi")
      .SetBounds(0.0, 1.0, 0.0, 1.0)
      .SetLayout(pangolin::LayoutEqual)
      .AddDisplay(d_cam1)
      .AddDisplay(d_img1)
      .AddDisplay(d_cam2)
      .AddDisplay(d_img2)
      .AddDisplay(d_cam3)
      .AddDisplay(d_cam4);

  const int width =  64;
  const int height = 48;
  unsigned char* imageArray = new unsigned char[3*width*height];
  //   pangolin::GlTexture imageTexture(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while( !pangolin::ShouldQuit() )
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Generate random image and place in texture memory for display
    setImageData(imageArray,3*width*height);
    // imageTexture.Upload(imageArray,GL_RGB,GL_UNSIGNED_BYTE);

    glColor3f(1.0,1.0,1.0);

    d_cam1.Activate(s_cam);
    pangolin::glDrawColouredCube();

    d_cam2.Activate(s_cam2);
    pangolin::glDrawColouredCube();

    d_cam3.Activate(s_cam);
    pangolin::glDrawColouredCube();

    d_cam4.Activate(s_cam2);
    pangolin::glDrawColouredCube();

    d_img1.Activate();
    glColor4f(1.0f,1.0f,1.0f,1.0f);
    // imageTexture.RenderToViewport();
    pangolin::glDrawColouredCube();

    d_img2.Activate();
    glColor4f(1.0f,1.0f,1.0f,1.0f);
    // imageTexture.RenderToViewport();
    pangolin::glDrawColouredCube();

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  delete[] imageArray;
}

void testMultiAgentDisplay() {
    pangolin::CreateWindowAndBind("test",1024,768);
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);
    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // Menu panel
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",false,true);
    pangolin::Var<bool> menuCamView("menu.Camera View",false,false);
    pangolin::Var<bool> menuTopView("menu.Top View",false,false);
    // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
    pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuStop("menu.Stop",false,false);
    pangolin::Var<bool> menuStepByStep("menu.Step By Step",false,true);  // false, true
    pangolin::Var<bool> menuStep("menu.Step",false,false);
    pangolin::Var<bool> menuShowOptLba("menu.Show LBA opt", false, true);

    // test : try multiview
    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000);
    pangolin::OpenGlRenderState s_cam(proj, pangolin::ModelViewLookAt(1,0.5,-2,0,0,0, pangolin::AxisY) );
    pangolin::OpenGlRenderState s_cam2(proj, pangolin::ModelViewLookAt(0,0,-2,0,0,0, pangolin::AxisY) );
    // Multiview
    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam1 = pangolin::Display("cam1")
    .SetAspect(1024.0f/768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::View& d_cam2 = pangolin::Display("cam2")
    .SetAspect(1024.0f/768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam2));

    pangolin::View& d_cam3 = pangolin::Display("cam3")
    .SetAspect(1024.0f/768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::View& d_cam4 = pangolin::Display("cam4")
    .SetAspect(1024.0f/768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam2));

    pangolin::View& d_img1 = pangolin::Display("img1")
    .SetAspect(1024.0f/768.0f);

    pangolin::View& d_img2 = pangolin::Display("img1")
    .SetAspect(1024.0f/768.0f);

    // LayoutEqual is an EXPERIMENTAL feature - it requires that all sub-displays
    // share the same aspect ratio, placing them in a raster fasion in the
    // viewport so as to maximise display size.
    pangolin::Display("multi")
        .SetBounds(0.0, 1.0, 0.0, 1.0)
        .SetLayout(pangolin::LayoutEqual)
        .AddDisplay(d_cam1)
        .AddDisplay(d_img1)
        .AddDisplay(d_cam2)
        .AddDisplay(d_img2)
        .AddDisplay(d_cam3)
        .AddDisplay(d_cam4);

    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( !pangolin::ShouldQuit() )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glColor3f(1.0,1.0,1.0);

        d_cam1.Activate(s_cam);
        pangolin::glDrawColouredCube();

        d_cam2.Activate(s_cam2);
        pangolin::glDrawColouredCube();

        d_cam3.Activate(s_cam);
        pangolin::glDrawColouredCube();

        d_cam4.Activate(s_cam2);
        pangolin::glDrawColouredCube();

        d_img1.Activate();
        glColor4f(1.0f,1.0f,1.0f,1.0f);
        pangolin::glDrawColouredCube();

        d_img2.Activate();
        glColor4f(1.0f,1.0f,1.0f,1.0f);
        pangolin::glDrawColouredCube();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
}

int main(/*int argc, char* argv[]*/)
{
//   testMultiTuto();
    testMultiAgentDisplay();
  return 0;
}