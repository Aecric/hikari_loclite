#include "ndt/ndt_omp.h"
#include "ndt/ndt_omp_impl.hpp"

template class pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;
template class pclomp::NormalDistributionsTransform<PointXYZIT, PointXYZIT>;
