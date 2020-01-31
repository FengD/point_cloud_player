#include "cloud_visualization.h"

CloudVisualization::CloudVisualization() : pcl::visualization::PCLVisualizer("view") {
  init();
}

CloudVisualization::~CloudVisualization() {

}

void CloudVisualization::init() {
	this->addCoordinateSystem(1.0);
}
