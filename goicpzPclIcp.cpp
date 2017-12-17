//
// Created by Murtuza Husain on 17/12/2017.
//

namespace goicpz {

    void PclRegister::registerFixedSurface(const std::str path) {
        // Load PLY data e.g. Stanford Bunny
        pcl::console::TicToc time;
        time.tic ();
        if (pcl::io::loadPLYFile (path, *cloud_in) < 0)
        {
            PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
            return (-1);
        }
        std::cout << "\nLoaded file " << path << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;
    }

    void PclRegister::applyTransformation() {
        // Defining a rotation matrix and translation vector
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

        // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
        double theta = M_PI / 8;  // The angle of rotation in radians
        transformation_matrix (0, 0) = cos (theta);
        transformation_matrix (0, 1) = -sin (theta);
        transformation_matrix (1, 0) = sin (theta);
        transformation_matrix (1, 1) = cos (theta);

        // A translation on Z axis (0.4 meters)
        transformation_matrix (2, 3) = 0.4;

        // Display in terminal the transformation matrix
        std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
        print4x4Matrix (transformation_matrix);

        // Executing the transformation
        pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
        *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use
    }

    void PclRegister::performIcp(int iterations) {
        // The Iterative Closest Point algorithm
        time.tic ();
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setMaximumIterations (iterations);
        icp.setInputSource (cloud_icp);
        icp.setInputTarget (cloud_in);
        icp.align (*cloud_icp);
        icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
        std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

        if (icp.hasConverged ()) {
            std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
            std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
            transformation_matrix = icp.getFinalTransformation ().cast<double>();
            print4x4Matrix (transformation_matrix);
        } else {
            PCL_ERROR ("\nICP has not converged.\n");
        }
    }

    PclRegister::print4x4Matrix (const Eigen::Matrix4d & matrix) {
        printf ("Rotation matrix :\n");
        printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
        printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
        printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
        printf ("Translation vector :\n");
        printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
    }

};