//
// Created by Murtuza Husain on 18/03/2018.
//

#include "GlobalRegister.h"
#include "math.t"

namespace goicpz {

    /**
     * W
     *
     * i == j
     * W_ij = sim(f(m_i),f(t_i)
     *
     * i != j
     * W_ij = alpha * g_d(m_i,m_j,t_i,t_j,sigma_d) + (1-alpha) * g_b(m_i,m_j,t_i,t_j,sigma_b)
     */
    void RigidRegister::buildAffinityMatrix() {

    }

    /**
     * g_x
     *
     * g_x(m_i,m_j,t_i,t_j,sigma_x) = 0.5 * (g(m_i,x_mi,t_i,x_ti,sigma_x) + g(m_i,x_mj,t_i,x_tj,sigma_x))
     */
    float RigidRegister::getContourConstraint(PointT mi, PointT mj, PointT ti, PointT tj) {
        //applyRigidityRegulator(mi, j) + applyRigidityRegulator()
    }

    /**
     * g
     *
     * g(m_i,m_j,t_i,t_j,sigma) = exp( (c(m_i,m_j,t_i,t_j) - 1)^2 / (2 * sigma^2) )
     */
    float RigidRegister::applyRigidityRegulator(PointT mi, PointT mj, PointT ti, PointT tj, float sigma) {
        return pow((correspondence(mi, mj, ti, tj) - 1), 2) / (2 * pow(sigma, 2));
    }

    /**
     * c
     *
     * c(m_i,m_j,t_i,t_j) = min [ (d(m_i,m_j) / (d(t_i,t_j) + epsi), (d(t_i,t_j) / (d(m_i,m_j) + epsi) ]
     */
    float RigidRegister::correspondence(PointT mi, PointT mj, PointT ti, PointT tj) {
        float a = geodesicDistance(mi, mj) / geodesicDistance(ti, tj) + _epsilon;
        float b = geodesicDistance(ti, tj) / geodesicDistance(mi, mj) + _epsilon;
        return std::min(a, b);
    }

    /**
     * d
     *
     * d(x_i,x_j)
     */
    float RigidRegister::geodesicDistance(PointT i, PointT j) {

    }
}