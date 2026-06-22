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
#include "base_type.h"


typedef std::vector<cv::Point2d> P2PMatchResult;
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
        virtual void fitTransmissionModel(std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &end_point, std::vector<double> &cov_t, std::vector<double> &cov_f, std::vector<double> &para)
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

        virtual void optimizeTransmissionModel(const P2PMatchResult &points, const OptimizationInput &input)
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

        virtual std::vector<std::vector<double>> getpara()
        {
            std::vector<std::vector<double>> empty;
            return empty;
        }
        

        virtual bool get_is_visual()
        {
            return false;
        }
        virtual bool set_first_time()
        {
            return false;
        }

    };

} // namespace lc_core

#endif // TRANSMISSION_MODEL_H