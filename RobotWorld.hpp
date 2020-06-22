#ifndef ROBOTWORLD_HPP_
#define ROBOTWORLD_HPP_

#include <vector>
#include <string>
#include "Config.hpp"
#include "ModelObject.hpp"
#include "Point.hpp"
#include "Message.hpp"
#include "MessageHandler.hpp"
#include "Observer.hpp"

namespace Model
{
class Robot;
typedef std::shared_ptr<Robot> RobotPtr;

class WayPoint;
typedef std::shared_ptr<WayPoint> WayPointPtr;

class Goal;
typedef std::shared_ptr<Goal> GoalPtr;

class Wall;
typedef std::shared_ptr<Wall> WallPtr;

class RobotWorld;
typedef std::shared_ptr<RobotWorld> RobotWorldPtr;

/**
	 *
	 */
class RobotWorld : public ModelObject, public Messaging::MessageHandler
{
public:
	/**
			 *
			 */
	static RobotWorld &getRobotWorld();
	/**
			 *
			 */
	RobotPtr newRobot(const std::string &aName = "New Robot",
					  const Point &aPosition = Point(-1, -1),
					  bool aNotifyObservers = true);
	/**
			 *
			 */
	WayPointPtr newWayPoint(const std::string &aName = "New WayPoint",
							const Point &aPosition = Point(-1, -1),
							bool aNotifyObservers = true);
	/**
			 *
			 */
	GoalPtr newGoal(const std::string &aName = "New Goal",
					const Point &aPosition = Point(-1, -1),
					bool aNotifyObservers = true);
	/**
			 *
			 */
	WallPtr newWall(const Point &aPoint1,
					const Point &aPoint2,
					bool aNotifyObservers = true);
	/**
			 *
			 */
	void deleteRobot(RobotPtr aRobot,
					 bool aNotifyObservers = true);
	/**
			 *
			 */
	void deleteWayPoint(WayPointPtr aWayPoint,
						bool aNotifyObservers = true);
	/**
			 *
			 */
	void deleteGoal(GoalPtr aGoal,
					bool aNotifyObservers = true);
	/**
			 *
			 */
	void deleteWall(WallPtr aWall,
					bool aNotifyObservers = true);
	/**
			 *
			 */
	RobotPtr getRobot(const std::string &aName) const;
	/**
			 *
			 */
	RobotPtr getRobot(const Base::ObjectId &anObjectId) const;
	/**
			 *
			 */
	WayPointPtr getWayPoint(const std::string &aName) const;
	/**
			 *
			 */
	WayPointPtr getWayPoint(const Base::ObjectId &anObjectId) const;
	/**
			 *
			 */
	GoalPtr getGoal(const std::string &aName) const;
	/**
			 *
			 */
	GoalPtr getGoal(const Base::ObjectId &anObjectId) const;
	/**
			 *
			 */
	WallPtr getWall(const Base::ObjectId &anObjectId) const;
	/**
			 *
			 */
	const std::vector<RobotPtr> &getRobots() const;
	/**
			 *
			 */
	const std::vector<WayPointPtr> &getWayPoints() const;
	/**
			 *
			 */
	const std::vector<GoalPtr> &getGoals() const;
	/**
			 *
			 */
	const std::vector<WallPtr> &getWalls() const;
	/**
			 *
			 */
	void populate(int aNumberOfWalls = 2);
	/**
			 *
			 */
	void populateScenario_1_lhs(int aNumberOfWalls);
	/**
			 *
			 */
	void populateScenario_1_rhs(int aNumberOfWalls);
	/**
			 *
			 */

	void populateScenario_2_lhs(int aNumberOfWalls);
	/**
			 *
			 */
	void populateScenario_2_rhs(int aNumberOfWalls);
	/**
			 *
			 */

	void populateScenario_3_lhs(int aNumberOfWalls);
	/**
			 *
			 */
	void populateScenario_3_rhs(int aNumberOfWalls);
	/**
			 *
			 */
	void unpopulate(bool aNotifyObservers = true);
	/**
			 *
			 * @param aKeepObjects Keep the objects with these ObjectIdsin the world
			 * @param aNotifyObservers
			 */
	void unpopulate(const std::vector<Base::ObjectId> &aKeepObjects,
					bool aNotifyObservers = true);
	/**
			 * @name Debug functions
			 */
	//@{
	/**
			 * Returns a 1-line description of the object
			 */
	virtual std::string asString() const;

	std::string asCopyString() const;
	/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
	virtual std::string asDebugString() const;
	//@}
	enum MessageType
	{
		EchoRequest,
		EchoResponse,
		CopyRobotsRequest,
		CopyRobotsResponse,
		CopyWorldRequest,
		CopyWorldResponse,
		StartRobotRequest,
		StartRobotResponse,
		StopRobotRequest,
		StopRobotResponse

	};

	enum copyType
	{
		Robot,
		WayPoint,
		Goal,
		Wall
	};

	void setWorld(std::string &messageBody);
	RobotWorldPtr getRobotWorldPtr()
	{
		return robotWorldPtr;
	};
	virtual void handleRequest(Messaging::Message &aMessage);
	/**
	 * This function is called by a ClientSession whenever a response to a previous request is received.
	 *
	 * @see Messaging::ResponseHandler::handleResponse( const Messaging::Message& aMessage)
	 */
	virtual void handleResponse(const Messaging::Message &aMessage);
	/**
			 *
			 */
	RobotWorld();
	/**
			 *
			 */
	virtual ~RobotWorld();
	bool isCommunicating() const
	{
		return communicating;
	}

	void startCommunicating();
	/**
			 * Connects to the ServerConnection that listens at port 12345 unless given
			 * an other port by specifying a command line argument -local_port=port
			 * and sends a message with messageType "1" and a body with "stop"
			 *
			 */
	void stopCommunicating();

protected:
private:
	/**
			 * The vectors are mutable to allow for lazy instantiation
			 */
	mutable std::vector<RobotPtr> robots;
	mutable std::vector<WayPointPtr> wayPoints;
	mutable std::vector<GoalPtr> goals;
	mutable std::vector<WallPtr> walls;

	bool communicating;
	bool changed = false;
	RobotWorldPtr robotWorldPtr;
};
} // namespace Model
#endif // ROBOTWORLD_HPP_
