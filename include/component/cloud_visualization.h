#include <pcl/visualization/pcl_visualizer.h>

class CloudVisualization : public pcl::visualization::PCLVisualizer {
 public:
  CloudVisualization();
  virtual ~CloudVisualization();
 private:
  void init();
};
