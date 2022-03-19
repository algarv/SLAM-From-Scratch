#include <measurement/measurement.hpp>
#include <ros/console.h>
#include <armadillo>

std::vector<circle> circle_detection(std::vector<cluster> clusters){

    int size = clusters.size();
    centroids.resize(size);
    
    std::vector<circle> circles;
    circles.resize(size);

    z_bar.resize(size);

    // ROS_WARN("Detecting circles from %d clusters", size);
    for (unsigned long int i=0; i < clusters.size(); i++){
        double cluster_size = clusters[i].pt.size();
        if (cluster_size>3 && ((clusters[i].pt[0].x != 0)||(clusters[i].pt[0].y != 0))){
            // ROS_WARN("Cluster %d is valid. Size: %d", i, cluster_size);
            double sum_x = 0;
            double sum_y = 0;
            double sum_z = 0;

            for (unsigned long int j=0; j < cluster_size; j++){
                sum_x += (clusters[i].pt[j].x);
                sum_y += (clusters[i].pt[j].y);
            }

            centroids[i] = {.x = sum_x/cluster_size, .y = sum_y/cluster_size};
            arma::mat Z (cluster_size,4, arma::fill::zeros);
            for (unsigned long int j=0; j < cluster_size; j++){
                clusters[i].pt[j].x -= (centroids[i].x);
                clusters[i].pt[j].y -= (centroids[i].y);    
                double x = clusters[i].pt[j].x;
                double y = clusters[i].pt[j].y;
                double z = pow((x),2)+ pow((y),2);
                sum_z += z;
                Z(j,0) = z;
                Z(j,1) = x;
                Z(j,2) = y;
                Z(j,3) = 1; 
            }
            // ROS_WARN("Z: ");
            // Z.print(std::cout);

            z_bar[i] = sum_z/cluster_size;

            arma::mat Z_T = Z.t();
            arma::mat M = (Z_T * Z)/cluster_size;
            // ROS_WARN("M: ");
            // M.print(std::cout);

            arma::mat H;
            H = arma::zeros(4,4);
            H(0,0) = 8 * z_bar[i];
            H(1,1) = 1;
            H(2,2) = 1;
            H(0,3) = 2;
            H(3,0) = 2;
            // ROS_WARN("H: ");
            // H.print(std::cout);

            arma::mat H_inv;
            H_inv = arma::zeros(4,4);
            H_inv(0,3) = 0.5;
            H_inv(1,1) = 1;
            H_inv(2,2) = 1;
            H_inv(3,0) = 0.5;
            H_inv(3,3) = -2 * z_bar[i];
            // ROS_WARN("H_inv: ");
            // H.print(std::cout);

            arma::mat U;
            arma::vec S_vec;
            arma::mat V;

            arma::svd(U, S_vec, V, Z);
            // ROS_WARN("U: ");
            // U.print(std::cout);
            // ROS_WARN("S_vec: ");
            // S_vec.print(std::cout);
            // ROS_WARN("V: ");
            // V.print(std::cout);

            arma::mat S_mat;
            S_mat = arma::zeros(S_vec.n_rows,S_vec.n_rows);
            float s;
            for (int i=0; i < S_vec.n_rows; i++){
                S_mat(i,i) = S_vec(i);
                if (i==0){
                    s = S_vec(i);
                }
                else{
                    if (S_mat(i,i) < s){
                        s = S_vec(i);
                    }
                }
            }
            // ROS_WARN("S_mat: ");
            // S_mat.print(std::cout);

            // ROS_WARN("s: %3.2f", s);

            arma::vec A(4);
            // ROS_WARN("s < %f",pow(10,-12));
            if (s<10e-12){
                A = V.col(3);
            }
            else{
                arma::mat Y;
                Y = V * S_mat * V.t();

                arma::mat Q;
                Q = Y * H.i() * Y;

                arma::colvec eigval;
                arma::mat eigvec;

                arma::eig_sym(eigval, eigvec, Q);
                // ROS_WARN("Eigvec: ");
                // eigvec.print(std::cout);
                // ROS_WARN("Eigval: ");
                // eigval.print(std::cout);

                int min_index = eigvec.n_cols - 1;
                for (int i=0; i<eigval.n_rows; i++){
                    if (eigval(i) > 0){
                        if (eigval(i) < eigval(min_index)){
                                min_index = i;
                        }
                    }
                }

                arma::vec e;
                e = eigvec.col(min_index);
                A = Y.i() * e;
            }
            // ROS_WARN("A: ");
            // A.print(std::cout);

            circles[i].a = (-1*A(1,0) / (2 * A(0,0))) + centroids[i].x;
            circles[i].b = (-1*A(2,0) / (2 * A(0,0))) + centroids[i].y;
            circles[i].R2 = (pow(A(1,0),2)+pow(A(2,0),2)-4*A(0,0)*A(3,0))/(4*pow(A(0,0),2));
        }
    }
    return circles;
}


std::vector<circle> circle_classification(std::vector<cluster> found_clusters, std::vector<circle> circles){
    std::vector<circle> confirmed_circles;
    // ROS_WARN("Unfiltered Cluster List Size: %d", found_clusters.size());
    for (unsigned long int i=0; i < found_clusters.size(); i++){
        unsigned long int cluster_size = found_clusters[i].pt.size();
        // ROS_WARN("Cluster Size: %ld", cluster_size);
        if (cluster_size>3){
            if (circles[i].R2 < (obj_radius * obj_radius)*1.2){
                std::vector<double> angles;
                angles.resize(cluster_size - 2); 
                double angle_sum = 0;
                point p1 = found_clusters[i].pt[0];
                point p2 = found_clusters[i].pt[cluster_size - 1];
                // ROS_WARN("Point 1: (%3.2f,%3.2f), Point 2: (%3.2f,%3.2f)", p1.x, p1.y, p2.x, p2.y);

                for (unsigned long int j=1; j < cluster_size - 1; j++){
                    point p3 = found_clusters[i].pt[j];
                    // ROS_WARN("Point 3: (%3.2f,%3.2f)", p3);

                    point a = {.x = p1.x - p3.x, .y = p1.y - p3.y};
                    point b = {.x = p2.x - p3.x, .y = p2.y - p3.y};
                    angles[j-1] = (a.x * b.x + a.y * b.y) / (sqrt(pow(a.x,2)+pow(a.y,2)) * sqrt(pow(b.x,2)+pow(b.y,2)));
                    // ROS_WARN("Angles: %3.2f", angles[j-1]);
                    angle_sum += angles[j-1];
                }

                double mean_angle = angle_sum/(cluster_size - 2);
                mean_angle = turtlelib::rad2deg(mean_angle);
                if (mean_angle < 0){
                    mean_angle *= -1;
                }
                double angle_StD = 0;
                // ROS_WARN("Mean Cluster Angles: %3.2f",mean_angle);
                if (mean_angle > 0 && mean_angle < 135){
                    // ROS_WARN("circles[i].R2 %3.2f",circles[i].R2);
                    if (circles[i].R2 != 0){
                        for (unsigned long int j=0; j < found_clusters[i].pt.size() - 2; j++){
                            angle_StD += pow((angles[i] - angle_StD),2);
                        }
                        angle_StD /= found_clusters[i].pt.size() - 2;
                        // ROS_WARN("STD: %3.2f",angle_StD);
                        if (angle_StD < .15){
                            confirmed_circles.push_back(circles[i]);
                            // ROS_WARN("Confirmed circle %d: R2 = %3.2f, a = %3.2f, b = %3.2f", i, circles[i].R2, circles[i].a, circles[i].b);
                        }
                    }
                }
            }
        } 
    }
    return confirmed_circles;
}