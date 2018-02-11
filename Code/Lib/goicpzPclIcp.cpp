//
// Created by Murtuza Husain on 17/12/2017.
//

#include "goicpzPclIcp.h"

namespace goicpz {

    void PclRegister::registerFixedSurface(std::string path)
    {
        time.tic();
        if (pcl::io::loadPLYFile(path, *cloud_in) < 0)
        {
            std::cerr << "Error loading cloud " << path << std::endl;
        }
        std::cout << "Cloud loaded from " << path << " in " << time.toc() << "ms" << std::endl;
    }

    void PclRegister::registerTargetSurface(std::string path)
    {
        time.tic();
        if (pcl::io::loadPLYFile(path, *cloud_tr) < 0)
        {
            std::cerr << "Error loading cloud " << path << std::endl;
        }
        std::cout << "Cloud loaded from " << path << " in " << time.toc() << "ms" << std::endl;
    }

    void PclRegister::performIcp()
    {
        // The Iterative Closest Point algorithm
        time.tic ();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans ( new pcl::PointCloud<pcl::PointXYZ> () );


        { // ICP registration
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

            icp.setInputSource ( cloud_in );
            icp.setInputTarget ( cloud_tr );

            // registration
            icp.align ( *cloud_source_trans );

            if (icp.hasConverged())
            {
                std::cout << "Converged. score =" << icp.getFitnessScore() << std::endl;

                Eigen::Matrix4f transformation = icp.getFinalTransformation();
                std::cout << transformation << std::endl;

                std::cout << "Converged in " << time.toc() << "ms" << std::endl;
            }
            else
            {
                std::cout << "Not converged." << std::endl;
            }
        }
    }

    void PclRegister::print4x4Matrix (const Eigen::Matrix4d & matrix)
    {
        printf ("Rotation matrix :\n");
        printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
        printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
        printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
        printf ("Translation vector :\n");
        printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
    }

};