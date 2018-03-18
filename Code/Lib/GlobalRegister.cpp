//
// Created by Murtuza Husain on 18/03/2018.
//

#include "GlobalRegister.h"

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
    void RigidRegister::getContourConstraint() {

    }

    /**
     * g
     *
     * g(m_i,m_j,t_i,t_j,sigma) = exp( (c(m_i,m_j,t_i,t_j) - 1)^2 / (2 * sigma^2) )
     */
    void RigidRegister::applyRigidityRegulator() {

    }

    /**
     * c
     *
     * c(m_i,m_j,t_i,t_j) = min [ (d(m_i,m_j) / (d(t_i,t_j) + epsi), (d(t_i,t_j) / (d(m_i,m_j) + epsi) ]
     */
    void RigidRegister::correspondece() {

    }

    /**
     * d
     *
     * d(x_i,x_j)
     */
    void RigidRegister::geodesicDistance(PointT i, PointT j) {

    }
}