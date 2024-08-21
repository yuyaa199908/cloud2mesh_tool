#ifndef PCL_REGION_GROWING_BALADO_H_
#define PCL_REGION_GROWING_BALADO_H_

#include <pcl/segmentation/region_growing.h>

namespace pcl
{
    template <typename PointT, typename NormalT>
    class PCL_EXPORTS RegionGrowingBalado : public RegionGrowing<PointT, NormalT> 
    {
        // public:           
        public:
            // using KdTree = pcl::search::Search<PointT>;
            // using KdTreePtr = typename KdTree::Ptr;
            // using Normal = pcl::PointCloud<NormalT>;
            // using NormalPtr = typename Normal::Ptr;
            // using PointCloud = pcl::PointCloud<PointT>;
        
            // using PCLBase <PointT>::input_;
            // using PCLBase <PointT>::indices_;
            // using PCLBase <PointT>::initCompute;
            // using PCLBase <PointT>::deinitCompute;

            double neighbour_distance_ {0.05};

            // KdTreePtr search_{nullptr};
            void setDistanceOfNeighbours (double neighbour_distance){
                neighbour_distance_ = neighbour_distance;
            }

            double getDistanceOfNeighbours(){
                return (neighbour_distance_);
            }
            
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
            void findPointNeighbours (){
                pcl::Indices neighbours;
                std::vector<float> distances;
                this->point_neighbours_.resize (this->input_->size (), neighbours);
                if (this->input_->is_dense)
                {
                    for (const auto& point_index: (*this->indices_))
                    {
                        neighbours.clear ();
                        // search_->nearestKSearch (point_index, neighbour_number_, neighbours, distances);
                        this->search_->radiusSearch (point_index, neighbour_distance_, neighbours, distances, this->neighbour_number_);  //error
                        this->point_neighbours_[point_index].swap (neighbours);
                    }
                }
                else
                {
                    std::cout << "input_->is_dense is False"<< std::endl;
                    for (const auto& point_index: (*this->indices_))
                    {
                        if (!pcl::isFinite ((*this->input_)[point_index])) continue;
                        neighbours.clear ();
                        // search_->nearestKSearch (point_index, neighbour_number_, neighbours, distances);
                        this->search_->radiusSearch (point_index, neighbour_distance_, neighbours, distances,this->neighbour_number_);
                        this->point_neighbours_[point_index].swap (neighbours);
                    }
                }
            }

    };
}

#endif