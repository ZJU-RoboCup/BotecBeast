// This file is part of RBDyn.
//
// Copyright (C) 2012 - 2017 CNRS-AIST JRL, CNRS-UM LIRMM
//
// RBDyn is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RBDyn is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with RBDyn.  If not, see <http://www.gnu.org/licenses/>.

// associated header
#include "Util.h"
// includes
// RBDyn
#include "RBDyn/IK.h"
#include "RBDyn/FK.h"
#include "RBDyn/FV.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"

//SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

namespace rbd
{

namespace {

struct CwiseRoundOp {
	CwiseRoundOp(const double& inf, const double& sup) : m_inf(inf), m_sup(sup) {}
	double operator()(const double& x) const { return x>m_inf && x<m_sup ? 0 : x; }
	double m_inf, m_sup;
};

} // anonymous

InverseKinematics::InverseKinematics(const MultiBody& mb, int ef_index):
	max_iterations_(ik::MAX_ITERATIONS),
	lambda_(ik::LAMBDA),
	threshold_(ik::THRESHOLD),
	almost_zero_(ik::ALMOST_ZERO),
	ef_index_(ef_index),
	jac_(mb, mb.body(ef_index).name()),
	svd_()
{
}

bool InverseKinematics::inverseKinematics(const MultiBody& mb, MultiBodyConfig& mbc,
                                          const sva::PTransformd& ef_target)
{
	int iter = 0;
	bool converged = false;
	int dof = 0;
	rbd::forwardKinematics(mb, mbc);
	Eigen::MatrixXd jacMat;
	Eigen::Vector6d v = Eigen::Vector6d::Ones();
	Eigen::Vector3d rotErr;
	Eigen::VectorXd res = Eigen::VectorXd::Zero(3);
	while( ! converged && iter < max_iterations_)
	{
		jacMat = jac_.jacobian(mb, mbc);
		//non-strict zeros in jacobian can be a problem...
		jacMat = jacMat.unaryExpr(CwiseRoundOp(-almost_zero_, almost_zero_));
		svd_.compute(jacMat, Eigen::ComputeThinU | Eigen::ComputeThinV);
		rotErr = sva::rotationError(mbc.bodyPosW[ef_index_].rotation(),
		                            ef_target.rotation());
		v << rotErr, ef_target.translation() - mbc.bodyPosW[ef_index_].translation();
		converged = v.norm() < threshold_;
		res = svd_.solve(v);

		dof = 0;
		for(auto index : jac_.jointsPath())
		{
			std::vector<double>& qi = mbc.q[index];
			for(auto &qv : qi)
			{
				qv += lambda_*res[dof];
				while(qv > Util::PI)
					qv -= 2*Util::PI;
				while(qv < -Util::PI)
					qv += 2*Util::PI;
				++dof;
			}
		}

		rbd::forwardKinematics(mb, mbc);
		rbd::forwardVelocity(mb, mbc);
		iter++;
	}
	return converged;
}

bool InverseKinematics::inverseKinematics(const MultiBody& mb, MultiBodyConfig& mbc,
                                          const sva::PTransformd& ef_target, const int mb_dof)
{
	// 欠驱动求逆解
	if(mb_dof==3)
	{
		int iter = 0;
		bool converged = false;
		int dof = 0;

		rbd::forwardKinematics(mb, mbc);
		Eigen::MatrixXd jacMat;
		Eigen::Matrix3d jacMat3d;
		Eigen::Vector3d v = Eigen::Vector3d::Ones();
		Eigen::VectorXd res = Eigen::VectorXd::Zero(3);

		while( ! converged && iter < max_iterations_)
		{
			jacMat = jac_.jacobian(mb, mbc);
			//non-strict zeros in jacobian can be a problem...
			jacMat = jacMat.unaryExpr(CwiseRoundOp(-almost_zero_, almost_zero_));
			jacMat3d = jacMat.block<3,3>(3,0);
			svd_.compute(jacMat3d, Eigen::ComputeThinU | Eigen::ComputeThinV);
			v << ef_target.translation() - mbc.bodyPosW[ef_index_].translation();
			converged = v.norm() < threshold_;
			res = svd_.solve(v);

			dof = 0;
			for(auto index : jac_.jointsPath())
			{
				std::vector<double>& qi = mbc.q[index];
				for(auto &qv : qi)
				{
					qv += lambda_*res[dof];
					// 转动的角度范围[-PI, PI]
					// 逆解使用的是弧度制
					while(qv > Util::PI)
						qv -= 2*Util::PI;
					while(qv < -Util::PI)
						qv += 2*Util::PI;
					++dof;
				}
			}

			rbd::forwardKinematics(mb, mbc);
			rbd::forwardVelocity(mb, mbc);
			iter++;
		}
		return converged;
	}
	// 全驱动求逆解
	else if (mb_dof==6)
	{
		inverseKinematics(mb, mbc, ef_target);
	}
}

bool InverseKinematics::sInverseKinematics(const MultiBody& mb, MultiBodyConfig& mbc,
					   const sva::PTransformd& ef_target)
{
	checkMatchQ(mb, mbc);
	checkMatchBodyPos(mb, mbc);
	checkMatchJointConf(mb, mbc);
	checkMatchParentToSon(mb, mbc);

	return inverseKinematics(mb, mbc, ef_target);
}

bool InverseKinematics::sInverseKinematics(const MultiBody& mb, MultiBodyConfig& mbc,
					   const sva::PTransformd& ef_target, const int mb_dof)
{
	checkMatchQ(mb, mbc);
	checkMatchBodyPos(mb, mbc);
	checkMatchJointConf(mb, mbc);
	checkMatchParentToSon(mb, mbc);

	return inverseKinematics(mb, mbc, ef_target, mb_dof);
}

} // namespace rbd
