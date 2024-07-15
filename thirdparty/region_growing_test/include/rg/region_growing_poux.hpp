#ifndef PCL_REGION_GROWING_POUX_H_
#define PCL_REGION_GROWING_POUX_H_

#include <pcl/segmentation/region_growing.h>

namespace pcl
{
    template <typename PointT, typename NormalT>
    class PCL_EXPORTS RegionGrowingPoux : public RegionGrowing<PointT, NormalT> 
    {
        // public:           
        public:
            void extractUnassigned (pcl::PointIndices& unassigned_){
                unassigned_.indices = *this->indices_;

                for (const auto& i_segment : this->clusters_)
                {
                    for (const auto& index : (i_segment.indices))
                    {
                        unassigned_.indices.erase(
                            std::remove(unassigned_.indices.begin(), unassigned_.indices.end(), index),
                            unassigned_.indices.end()
                        );
                    }
                }
            }

        protected:

    };
}

#endif