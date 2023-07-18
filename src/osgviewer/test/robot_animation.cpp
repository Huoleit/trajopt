#include <osg/AnimationPath>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/io_utils>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
using namespace osg;

osgViewer::Viewer viewer;

osg::AnimationPath* createAnimationPath(float radius, float time) {
  osg::ref_ptr<osg::AnimationPath> path = new osg::AnimationPath;
  path->setLoopMode(osg::AnimationPath::LOOP);

  unsigned int numSamples = 32;
  float delta_yaw = 2.0f * osg::PI / ((float)numSamples - 1.0f);
  float delta_time = time / (float)numSamples;
  for (unsigned int i = 0; i < numSamples; ++i) {
    float yaw = delta_yaw * (float)i;
    osg::Vec3 pos(sinf(yaw) * radius, cosf(yaw) * radius, 0.0f);
    osg::Quat rot(-yaw, osg::Z_AXIS);
    path->insert(delta_time * (float)i, osg::AnimationPath::ControlPoint(pos, rot));
  }
  return path.release();
}

int main(int argc, char* argv[]) {
  osg::ref_ptr<osg::MatrixTransform> root = new osg::MatrixTransform;
  viewer.setUpViewInWindow(0, 0, 640, 480);
  viewer.realize();
  ref_ptr<Camera> cam = viewer.getCamera();
  ref_ptr<Geode> geode = new Geode;
  root->addChild(geode.get());

  for (int i = 0; i < 10; i++) {
    osg::Sphere* sphere = new osg::Sphere(Vec3f(i + 1, 10, 0), .1 * i);
    osg::ShapeDrawable* sphereDrawable = new osg::ShapeDrawable(sphere);
    geode->addDrawable(sphereDrawable);
  }

  osg::ref_ptr<osg::AnimationPathCallback> apcb = new osg::AnimationPathCallback;
  apcb->setAnimationPath(createAnimationPath(50.0f, 6.0f));
  root->setUpdateCallback(apcb.get());

  ref_ptr<osgGA::TrackballManipulator> manip = new osgGA::TrackballManipulator();

  viewer.setCameraManipulator(manip);

  viewer.setSceneData(root.get());
  cam->setViewMatrixAsLookAt(Vec3f(10, 0, 0), Vec3f(10, 1, 0), Vec3f(0, 0, 1));
  // manip->setHomePosition(Vec3f(10,0,0), Vec3f(11,1,0), Vec3f(10,0,1));
  //  cam->setProjectionMatrixAsPerspective(49,640/480., .1, 10);

  viewer.run();
}
