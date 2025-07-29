/**
 * @file   transmissionModel.h
 * @brief  This file is the base class of transmission model.
 * @author Yipeng Zhao
 * @date   2024-10
 */

#ifndef TRANSMISSION_MODEL_H
#define TRANSMISSION_MODEL_H

#include <vector>
#include <random>
#include <Eigen/Dense>
#include "glog/logging.h"
#include "matcher.h"
#include "base_type.h"

namespace lc_core
{

    class TransmissionModel
    {
    public:
        /// @brief  Constructor
        TransmissionModel() = default;

        /// @brief  Destructor
        virtual ~TransmissionModel() = default;

        /// @brief  Fit the transmission model based on the given points
        virtual void fitTransmissionModel(std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &end_point)
        {
            LOG(INFO) << "Choose one transmission model";
        }

        // /// @brief  Generate the points based on the transmission model
        // virtual std::vector<Eigen::Vector3d> generatePoints(const double &x_start, const double &x_end, const double &x_step);

        /// @brief  Generate single point based on the transmission model
        virtual Eigen::Vector3d generateSinglePoint(const double &x)
        {
            LOG(INFO) << "Choose one transmission model";
            return Eigen::Vector3d(0, 0, 0);
        }

        /// @brief  Optimizate transmission model
        virtual void optimizeTransmissionModel(const P2LMatchResult &lines, const OptimizationInput &input, int y_optimize = 0)
        {
            LOG(INFO) << "Choose one transmission model";
        }

        virtual void optimizeTransmissionModel(const P2PMatchResult &points, const OptimizationInput &input, int y_optimize = 0, int time = 1)
        {
            LOG(INFO) << "Choose one transmission model";
        }

        /// @brief  Dark optimize interface
        virtual void optimizeTransmissionModelDark(const Eigen::Vector3d& end_point)
        {
            LOG(INFO) << "Choose one transmission model";
        }


        /// @brief  RANSAC fit x-y line
        virtual std::pair<double, double> ransacFitLine(std::vector<Eigen::Vector3d> &points);


    };

} // namespace lc_core

#endif // TRANSMISSION_MODEL_H