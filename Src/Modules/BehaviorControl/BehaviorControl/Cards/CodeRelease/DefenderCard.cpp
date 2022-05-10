/**
 * @file DefenderCard.cpp
 *
 * This file implements a basic striker behavior for the code release.
 * Normally, this would be decomposed into at least
 * - a ball search behavior card
 * - a skill for getting behind the ball
 *
 * @author Dap y Mia
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Libraries/LibCodeRelease.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"

CARD(DefenderCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(LibCodeRelease),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(1000) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
    (Angle)(5_deg) ballAlignThreshold,
    (float)(500.f) ballNearThreshold,
    (Angle)(10_deg) angleToGoalThreshold,
    (float)(400.f) ballAlignOffsetX,
    (float)(100.f) ballYThreshold,
    (Angle)(2_deg) angleToGoalThresholdPrecise,
    (float)(150.f) ballOffsetX,
    (Rangef)({140.f, 170.f}) ballOffsetXRange,
    (float)(40.f) ballOffsetY,
    (Rangef)({20.f, 50.f}) ballOffsetYRange,
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,
  }),
});

class DefenderCard : public DefenderCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  option
  {
    theActivitySkill(BehaviorStatus::Defender);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto turnToBall;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(turnToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
          goto walkToBall;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
      }
    }

    state(geneticDefense) 
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(!theLibCodeRelease.theRivalIsCloserToTheBall && theLibCodeRelease.closerToTheBall)
          goto clearingBall;
        if((theLibCodeRelease.theRivalIsCloserToTheBall && theFieldBall.positionRelative.norm() < (400.f) && theLibCodeRelease.closerToTheBall)
        || (theLibCodeRelease.numberOfDefences == 1 && state_time > 10000 && theLibCodeRelease.theRivalIsCloserToTheBall))
          goto attackForDefense;
        if(std::abs(theFieldBall.positionRelative.x()) > 500)
          goto followFromDistance;
      }

      action
      {
        float ballWeight = 1.2f;

        if(theLibCodeRelease.closerToTheBall && state_time > 800 && theLibCodeRelease.numberOfDefenders > 1 && theLibCodeRelease.theRivalIsCloserToTheBall)
          if(std::abs(theFieldBall.positionRelative.y()) < -5 && theLibCodeRelease.defenderLefter)
            ballWeight = (float)(ballWeight + (state_time / 1000));
          if(std::abs(theFieldBall.positionRelative.y()) > 5 && theLibCodeRelease.defenderRighter)
            ballWeight = (float)(ballWeight + (state_time / 1000));

        theLookForwardSkill();
      }
    }

    state(clearingBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theLibCodeRelease.theRivalIsCloserToTheBall && theLibCodeRelease.numberOfDefences > 1 && std::abs(theFieldBall.positionRelative.norm()) < 400.f && theLibCodeRelease.closerToTheBall)
          goto attackForDefense;
        if(std::abs(theFieldBall.positionRelative.norm()) < 500.f)
          goto shoot;
        if(!theLibCodeRelease.closerToTheBall || theLibCodeRelease.theRivalIsCloserToTheBall)
          goto geneticDefense;
      }

      action 
      {

      }
    }

    state(attackForDefense)
    {
      transition
      {
        
      }
    }

    state(shoot) 
    {
      transition
      {
        if(theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theLibCodeRelease.theRivalIsCloserToTheBall && theLibCodeRelease.numberOfDefences > 1 && std::abs(theFieldBall.positionRelative.norm()) < 400.f && theLibCodeRelease.closerToTheBall)
          goto attackForDefense;
        if(std::abs(theFieldBall.positionRelative.norm()) > 600)
          goto geneticDefense;
        if(!theLibCodeRelease.closerToTheBall)
          goto geneticDefense;
      }

      action
      {
        Vector2f target = Vector2f(4500.f, 0.f);

        if(!theLibCodeRelease.shootToGoal && false)
          target = theLibCodeRelease.bestTeammateForPass();
      }
    }

    state(followFromDistance)
    {
      transition
      {
        if(theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theLibCodeRelease.theRivalIsCloserToTheBall && theLibCodeRelease.numberOfDefences > 1 && std::abs(theFieldBall.positionRelative.norm()) < 400.f && theLibCodeRelease.closerToTheBall)
          goto attackForDefense;
        if(!theLibCodeRelease.theRivalIsCloserToTheBall && theLibCodeRelease.closerToTheBall)
          goto clearingBall;
        if(std::abs(theFieldBall.positionRelative.x()) < 450)
          goto geneticDefense;
      }

      action
      {
        theLookForwardSkill();

        Vector2f going2 = {-2000.f,std::abs(theFieldBall.positionRelative.y())};

        if(theLibCodeRelease.numberOfDefenders==3){
          if(theLibCodeRelease.defenderLefter && theRobotPose.translation.y() < 2700 && theRobotPose.translation.y() > -1700)
            going2 = {-3000.f,std::abs(theFieldBall.positionRelative.y()) + 700.f};
          else if(theLibCodeRelease.defenderRighter && theRobotPose.translation.y() > -2700 && theRobotPose.translation.y() < 1700)
            going2 = {-3000.f, std::abs(theFieldBall.positionRelative.y()) - 700.f};
        }
        else {
          if(theLibCodeRelease.defenderLefter && theRobotPose.translation.y() < 2700 && theRobotPose.translation.y() > -1700)
          going2 = {-2000.f,std::abs(theFieldBall.positionRelative.y())};
        else if(theLibCodeRelease.defenderRighter && theRobotPose.translation.y() > -2700 && theRobotPose.translation.y() < 1700)
          going2 = {-3000.f, std::abs(theFieldBall.positionRelative.y()) - 700.f};
        }
      }
    }

    state(walkToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto alignToGoal;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
      }
    }

    state(goBackHome)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto geneticDefense;
      }

      action
      {
        Vector2f going2;

        if(theLibCodeRelease.numberOfDefenders == 1)
          going2 = (Vector2f){-2400,0};
        else if(theLibCodeRelease.numberOfDefenders == 2)
          going2 = (theLibCodeRelease.getNumberWithinRole() == 2) ? (Vector2f){-2400,500} : (Vector2f){-2400,-500};
        else if(theLibCodeRelease.numberOfDefenders == 3)
          going2 = (theLibCodeRelease.getNumberWithinRole() == 2) ? (Vector2f){-2400,700} : (theLibCodeRelease.getNumberWithinRole() == 3) ? (Vector2f){-2400,-700} : (Vector2f){-2400,0};
      }
    }

    state(alignToGoal)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
          goto alignBehindBall;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
      }
    }

    state(alignBehindBall)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToGoal) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
          goto kick;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }

    state(kick)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }
};

MAKE_CARD(DefenderCard);
