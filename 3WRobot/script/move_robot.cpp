#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class MoveRobot : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr)
    {
        this->ThreeWRobotModel = _parent;

	rSpeed = 0.75;

	gazebo::math::Quaternion quat =ThreeWRobotModel->GetWorldPose().rot; 
       	gazebo::math::Vector3 xAxisUnitVec(1,0,0);
	
	vecDir = quat.RotateVector(xAxisUnitVec);
	vecDir = vecDir.GetRounded();
	
	vecDir =  vecDir * rSpeed;
	std::cout << "Linear Velocity of Robot "<<ThreeWRobotModel->GetName()<<" set to : "<<vecDir<< std::endl;

        this->updateEvent = event::Events::ConnectWorldUpdateBegin(
         std::bind(&MoveRobot::OnUpdate, this));

	this->bReverse = false;
	this->ThreeWRobotModel->SetLinearVel(vecDir.Ign());
    }

    public: void OnUpdate()
    {
	physics::WorldPtr world  = ThreeWRobotModel->GetWorld(); 

	for (unsigned int i = 0; i < world->GetModelCount(); ++i)
  	{
    		physics::ModelPtr model = world->GetModel(i);
    		if ((ThreeWRobotModel->GetName() == "3WRobot" && model->GetName() == "person_walking") || 
		    (ThreeWRobotModel->GetName() == "3WRobot_0" &&  model->GetName() == "person_standing"))
    		{
      		   gazebo::math::Vector3 offset = model->GetWorldPose().pos - ThreeWRobotModel->GetWorldPose().pos;
      		   double modelDist = offset.GetLength();
		   if (modelDist < 2.0)
		   {
			this->bReverse = true;
			this->ThreeWRobotModel->SetLinearVel((-vecDir).Ign());			
		   }
		   else if (modelDist > 3.0 && this->bReverse == true)
		   {
			this->bReverse = false;
			this->ThreeWRobotModel->SetLinearVel(vecDir.Ign());			
		   }		       
      		}
    	}	
    }

    private: physics::ModelPtr ThreeWRobotModel;
    private: event::ConnectionPtr updateEvent;
    private: bool bReverse;  
    private: double rSpeed;
    private: gazebo::math::Vector3 vecDir;

  };

   GZ_REGISTER_MODEL_PLUGIN(MoveRobot)
}

