
#include <legged_locomotion_mpc/trajectory_planners/JointTrajectoryPlanner.hpp>


#include <floating_base_model/AccessHelperFunctions.hpp>

#include <ocs2_core/misc/LoadData.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace legged_locomotion_mpc
{
  namespace planners
  {
    using namespace ocs2;
    using namespace floating_base_model;
    using namespace multi_end_effector_kinematics;
    using namespace legged_locomotion_mpc::locomotion;

    // Helper function for creating end effector matrix from normal and base heading
    matrix3_t getEndEffectorRotationMatrix(const vector3_t& endEffectorNormal, 
      const vector3_t& baseHeadingVector)
    {
      /** Heading vector (X) same as base, but normal to sufrace normal 
       * (without surface normal part)
       */
      const vector3_t endEffectorHeading = baseHeadingVector - (baseHeadingVector.dot(
        endEffectorNormal)) * endEffectorNormal;

      matrix3_t rotationMatrix;
      rotationMatrix.col(0) = endEffectorHeading;
      // Get Y vector from cross product
      rotationMatrix.col(1) = endEffectorNormal.cross(endEffectorHeading);
      rotationMatrix.col(2) = endEffectorNormal;

      return rotationMatrix;
    }

    JointTrajectoryPlanner::JointTrajectoryPlanner(
      FloatingBaseModelInfo modelInfo,
      MultiEndEffectorKinematics&& kinematicsSolver): 
        modelInfo_(std::move(modelInfo)),
        kinematicsSolver_(std::move(kinematicsSolver)),
        jointPositionUpperLimits_(kinematicsSolver_.getPinocchioModel().lowerPositionLimit),
        jointPositionLowerLimits_(kinematicsSolver_.getPinocchioModel().upperPositionLimit),
        jointVelocityLimits_(kinematicsSolver_.getPinocchioModel().velocityLimit)
      {
        if(jointPositionUpperLimits_.size() != modelInfo_.actuatedDofNum)
        {
          std::stringstream stringStream;
          stringStream << "[JointTrajectoryPlanner]: Size of joint upper limits is " <<  jointPositionUpperLimits_.size() 
            << ", should be " << modelInfo_.actuatedDofNum << "!";
          throw std::invalid_argument(stringStream.str());
        }

        if(jointPositionLowerLimits_.size() != modelInfo_.actuatedDofNum)
        {
          std::stringstream stringStream;
          stringStream << "[JointTrajectoryPlanner]: Size of joint lower limits is " <<  jointPositionLowerLimits_.size() 
            << ", should be " << modelInfo_.actuatedDofNum << "!";
          throw std::invalid_argument(stringStream.str());
        }

        if(jointVelocityLimits_.size() != modelInfo_.actuatedDofNum)
        {
          std::stringstream stringStream;
          stringStream << "[JointTrajectoryPlanner]: Size of joint velocity limits is " << jointVelocityLimits_.size() 
            << ", should be " << modelInfo_.actuatedDofNum << "!";
          throw std::invalid_argument(stringStream.str());
        }
      }
    
    void JointTrajectoryPlanner::updateTrajectory(
      const SystemObservation& currentObservation,
      TargetTrajectories& targetTrajectories, 
      const position_trajectories& endEffectorPositionTrajectories,
      const velocity_trajectories& endEffectorVelocityTrajectories, 
      const normal_trajectories& endEffectorNormalTrajectories)
    {
      assert(endEffectorPositionTrajectories.size() == targetTrajectories.timeTrajectory.size());
      assert(endEffectorVelocityTrajectories.size() == targetTrajectories.timeTrajectory.size());
      assert(endEffectorNormalTrajectories.size() == targetTrajectories.timeTrajectory.size());

      const auto& currentState = currentObservation.state;
      const auto& currentInput = currentObservation.input;

      const size_t trajectorySize = targetTrajectories.timeTrajectory.size();
      auto& currentOptimalState = targetTrajectories.stateTrajectory[0];
      auto& currentOptimalInput = targetTrajectories.inputTrajectory[0];

      access_helper_functions::getJointPositions(currentOptimalState, modelInfo_) 
        = access_helper_functions::getJointPositions(currentState, modelInfo_);
      
      access_helper_functions::getJointVelocities(currentOptimalInput, modelInfo_) 
        = access_helper_functions::getJointVelocities(currentInput, modelInfo_);
      
      for(size_t i = 1; i < trajectorySize; ++i)
      {
        const scalar_t currentTime = targetTrajectories.timeTrajectory[i];
        const scalar_t previousTime = targetTrajectories.timeTrajectory[i - 1];

        const scalar_t deltaTime = currentTime - previousTime;

        const std::vector<vector3_t>& newEndEffectorPositions = 
          endEffectorPositionTrajectories[i];
        const std::vector<vector3_t>& newEndEffectorVelocities = 
          endEffectorVelocityTrajectories[i];
        const std::vector<vector3_t>& newEndEffectorNormals = 
          endEffectorNormalTrajectories[i];

        const auto& previousState = targetTrajectories.stateTrajectory[i - 1];
        const auto& previousInput = targetTrajectories.inputTrajectory[i - 1];

        const vector_t previousJointPositions = 
          access_helper_functions::getJointPositions(previousState, 
            modelInfo_);

        const vector_t previousJointVelocities = 
          access_helper_functions::getJointVelocities(previousInput, 
            modelInfo_);

        // Extrapolate using previous joint velocity
        const vector_t extrapolatedJointPositions = previousJointPositions + 
            previousJointVelocities * deltaTime;
            
        vector_t& newState = targetTrajectories.stateTrajectory[i];
        vector_t& newInput = targetTrajectories.inputTrajectory[i];

        const vector6_t& newBasePose = 
          access_helper_functions::getBasePose(newState, modelInfo_);
        
        const vector6_t& newBaseVelocity = 
          access_helper_functions::getBaseVelocity(newState, modelInfo_);

        auto newJointPositions = 
          access_helper_functions::getJointPositions(newState, modelInfo_);

        auto newJointVelocities = 
          access_helper_functions::getJointVelocities(newInput, modelInfo_);
        
        newJointPositions = computeJointPositions(extrapolatedJointPositions, newBasePose, 
          newEndEffectorPositions, newEndEffectorNormals);
          
        newJointVelocities = computeJointVelocities(newJointPositions, newBasePose,
          newBaseVelocity, newEndEffectorVelocities);
      }
    }

    vector_t JointTrajectoryPlanner::computeJointPositions(
      const vector_t& actualJointPositions,
      const vector6_t& basePose,  
      const std::vector<vector3_t>& endEffectorPositions, 
      const std::vector<vector3_t>& endEffectorNormals)
    {
      std::vector<vector3_t> relativeEndEffectorPositions(modelInfo_.numThreeDofContacts);
      std::vector<pinocchio::SE3> relativeEndEffectorTransforms(
        modelInfo_.numSixDofContacts);

      const vector3_t baseOrientationZyx = basePose.block<3, 1>(3, 0);
      const matrix3_t worldToBaseRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        baseOrientationZyx).transpose();

      for(size_t i = 0; i < modelInfo_.numThreeDofContacts; ++i)
      {
        relativeEndEffectorPositions[i].noalias() = worldToBaseRotationMatrix * 
          (endEffectorPositions[i] - basePose.block<3, 1>(0, 0));
      }

      for(size_t i = 0; i < modelInfo_.numSixDofContacts; ++i)
      {
        // Build end-effector -> world rotation matrix 

        const vector3_t normal = worldToBaseRotationMatrix * endEffectorNormals[i 
          + modelInfo_.numThreeDofContacts];

        const vector3_t baseHeading{1.0, 0.0, 0.0};

        const matrix3_t endEffectorToBaseRotation = getEndEffectorRotationMatrix(normal, 
          baseHeading);

        relativeEndEffectorTransforms[i].translation().noalias() = worldToBaseRotationMatrix * 
          (endEffectorPositions[i + modelInfo_.numThreeDofContacts] - basePose.block<3, 1>(0, 0));
        relativeEndEffectorTransforms[i].rotation() = endEffectorToBaseRotation;
      }

      vector_t newJointPositions;

      const ReturnStatus returnStatus = kinematicsSolver_.calculateJointPositions(
        actualJointPositions, relativeEndEffectorPositions, relativeEndEffectorTransforms, 
        newJointPositions);
      
      if(returnStatus.success) return newJointPositions;
      else
      {
        switch (returnStatus.flag)
        {
          case TaskReturnFlag::IN_PROGRESS:
            return newJointPositions;
            break;

          case TaskReturnFlag::SMALL_STEP_SIZE:
            return newJointPositions;
            break;

          case TaskReturnFlag::NEW_POSITION_OUT_OF_BOUNDS:
            for(size_t i = 0; i < modelInfo_.actuatedDofNum; ++i)
            {
              if(newJointPositions[i] > jointPositionUpperLimits_[i])
              {
                newJointPositions[i] = jointPositionUpperLimits_[i];
              }
              else if(newJointPositions[i] < jointPositionLowerLimits_[i])
              {
                newJointPositions[i] = jointPositionLowerLimits_[i];
              }
            }
            return newJointPositions;
            break;

          default:
            return newJointPositions;
            break;
        }
      }
    }

    vector_t JointTrajectoryPlanner::computeJointVelocities(
      const vector_t& actualJointPositions, const vector6_t& basePose, 
      const vector6_t& baseVelocity, const std::vector<vector3_t>& endEffectorVelocities)
    {
      std::vector<vector3_t> relativeEndEffectorVelocities(modelInfo_.numThreeDofContacts);

      std::vector<pinocchio::Motion> relativeEndEffectorTwists(
        modelInfo_.numSixDofContacts);

      const vector3_t baseOrientationZyx = basePose.block<3, 1>(3, 0);
      const matrix3_t worldToBaseRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        baseOrientationZyx).transpose();

      std::vector<vector3_t> relativeEndEffectorPositions(modelInfo_.numThreeDofContacts);

      std::vector<pinocchio::SE3> relativeEndEffectorTransforms(
        modelInfo_.numSixDofContacts);

      kinematicsSolver_.calculateEndEffectorPoses(actualJointPositions, 
        relativeEndEffectorPositions, relativeEndEffectorTransforms);

      const vector3_t& baseLinearVelocity = baseVelocity.block<3, 1>(0, 0);
      const vector3_t& baseAngularVelocity = baseVelocity.block<3, 1>(3, 0);
      
      for(size_t i = 0; i < modelInfo_.numThreeDofContacts; ++i)
      {
        relativeEndEffectorVelocities[i].noalias() = worldToBaseRotationMatrix * 
          (endEffectorVelocities[i]) - baseLinearVelocity - 
          baseAngularVelocity.cross(relativeEndEffectorPositions[i]);
      }

      for(size_t i = 0; i < modelInfo_.numSixDofContacts; ++i)
      {
        relativeEndEffectorTwists[i].linear().noalias() = worldToBaseRotationMatrix * 
          (endEffectorVelocities[i + modelInfo_.numThreeDofContacts]) - baseLinearVelocity 
          - baseAngularVelocity.cross(relativeEndEffectorTransforms[i].translation());
        relativeEndEffectorTwists[i].angular().noalias() = -baseAngularVelocity;
      }

      vector_t jointVelocities;

      const ReturnStatus returnStatus = kinematicsSolver_.calculateJointVelocities(
        actualJointPositions, relativeEndEffectorVelocities, 
        relativeEndEffectorTwists, jointVelocities);
      
      if(!returnStatus.success)
      {

        jointVelocities = jointVelocities.binaryExpr(jointVelocityLimits_, 
          [](scalar_t lhs, scalar_t rhs)
          {
            if(lhs > rhs) return rhs;
            if(lhs < -rhs) return -rhs;
          });
      }
      return jointVelocities;
    }

    KinematicsModelSettings loadKinematicsModelSettings(
      const std::string& filename, const std::string& fieldName, bool verbose)
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(filename, pt);

      KinematicsModelSettings settings;

      if(verbose) 
      {
        std::cerr << "\n #### Legged Locomotion MPC Inverse Kinematics Model Settings:";
        std::cerr << "\n #### =============================================================================\n";
      }

      loadData::loadPtreeValue(pt, settings.baseLinkName, fieldName + ".baseLinkName", verbose);
      
      loadData::loadStdVector(filename, fieldName + ".endEffectorThreeDofNames", settings.threeDofEndEffectorNames, verbose);
    
      loadData::loadStdVector(filename, fieldName + ".endEffectorSixDofNames", settings.sixDofEndEffectorNames, verbose);

      if(verbose) 
      {
        std::cerr << " #### =============================================================================" <<
        std::endl;
      }

      return settings;
    }

    InverseSolverSettings loadInverseSolverSettings(const std::string& filename, 
      const std::string& fieldName, bool verbose)
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(filename, pt);

      InverseSolverSettings settings;

      if(verbose) 
      {
        std::cerr << "\n #### Legged Locomotion MPC Inverse Kinematics Solver Settings:";
        std::cerr << "\n #### =============================================================================\n";
      }

      loadData::loadPtreeValue(pt, settings.maxIterations, fieldName + ".maxIterations", verbose);
      if(settings.maxIterations < 0)
      {
        throw std::invalid_argument("[JointTrajectoryPlanner]: Max iterations smaller than 0!");
      }

      loadData::loadPtreeValue(pt, settings.tolerance, fieldName + ".tolerance", verbose);
      if(settings.tolerance < 0.0)
      {
        throw std::invalid_argument("[JointTrajectoryPlanner]: Tolerance smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.minimumStepSize, fieldName + ".minimumStepSize", verbose);
      if(settings.minimumStepSize < 0.0)
      {
        throw std::invalid_argument("[JointTrajectoryPlanner]: Minimum step size smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.dampingCoefficient, fieldName + ".dampingCoefficient", verbose);
      if(settings.dampingCoefficient < 0.0)
      {
        throw std::invalid_argument("[JointTrajectoryPlanner]: Damping coefficient smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.stepCoefficient, fieldName + ".stepCoefficient", verbose);
      if(settings.stepCoefficient < 0.0)
      {
        throw std::invalid_argument("[JointTrajectoryPlanner]: Step coefficient smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.singularityThreshold, fieldName + ".singularityThreshold", verbose);
      if(settings.singularityThreshold < 0.0)
      {
        throw std::invalid_argument("[JointTrajectoryPlanner]: Singularity threshold smaller than 0.0!");
      }

      if(verbose) 
      {
        std::cerr << " #### =============================================================================" <<
        std::endl;
      }

      return settings;
    }

    std::string loadInverseSolverName(const std::string& filename, 
      const std::string& fieldName, bool verbose)
    {

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(filename, pt);

      std::string solverName;

      if(verbose) 
      {
        std::cerr << "\n #### Legged Locomotion MPC Inverse Kinematics Solver Name:";
        std::cerr << "\n #### =============================================================================\n";
      }

      loadData::loadPtreeValue(pt, solverName, fieldName + "", verbose);

      if(verbose) 
      {
        std::cerr << " #### =============================================================================" <<
        std::endl;
      }

      return solverName;
    }
  } // namespace planners
} // namespace legged_locomotion_mpc
