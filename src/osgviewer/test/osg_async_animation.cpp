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

osg::AnimationPath* createAnimationPath(int numSteps) {
  osg::ref_ptr<osg::AnimationPath> path = new osg::AnimationPath;
  path->setLoopMode(osg::AnimationPath::LOOP);

  double delta_time = 0.1;
  double speed = 0.5;
  for (unsigned int i = 0; i < numSteps; ++i) {
    osg::Vec3 pos(delta_time * (float)i * speed, 0.0f, 0.0f);
    osg::Quat rot(0, osg::Z_AXIS);
    path->insert(delta_time * (float)i, osg::AnimationPath::ControlPoint(pos, rot));
  }
  return path.release();
}

int main(int argc, char* argv[]) {
  viewer.setUpViewInWindow(0, 0, 1280, 960);
  osgGA::TrackballManipulator* manip = new osgGA::TrackballManipulator();
  manip->setHomePosition(Vec3f(0, 0, 10), Vec3f(0, 0, 0), Vec3f(-1, 0, 0));
  viewer.setCameraManipulator(manip);

  Group* root = new Group;

  // sphere 1
  Geode* geode = new Geode;
  geode->addDrawable(new ShapeDrawable(new Sphere(Vec3f(0, 0, 0), .5)));
  MatrixTransform* node = new MatrixTransform;
  node->addChild(geode);

  osg::AnimationPathCallback* apcb = new osg::AnimationPathCallback;
  apcb->setAnimationPath(createAnimationPath(30));
  node->setUpdateCallback(apcb);

  root->addChild(node);

  // sphere 2
  geode = new Geode;
  geode->addDrawable(new ShapeDrawable(new Sphere(Vec3f(0, 1, 0), .5)));
  node = new MatrixTransform;
  node->addChild(geode);

  apcb = new osg::AnimationPathCallback;
  apcb->setAnimationPath(createAnimationPath(50));
  node->setUpdateCallback(apcb);

  root->addChild(node);

  viewer.setSceneData(root);
  Camera* cam = viewer.getCamera();

  cam->setViewMatrixAsLookAt(Vec3f(0, 0, 5), Vec3f(0, 0, 0), Vec3f(-1, 0, 0));
  //  cam->setProjectionMatrixAsPerspective(49,640/480., .1, 10);

  viewer.run();
}
