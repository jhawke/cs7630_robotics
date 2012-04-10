/*!
 * \file cba.h
 * \brief Confidence-Based Autonomy (CBA) implementation for ROS
 *
 * cba provides and implementation of the Learning from Demonstration algorithm Confidence-Based Autonomy (CBA) built for ROS.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date December 19, 2011
 */

#ifndef CBA_H_
#define CBA_H_

#include <ANN/ANN.h>
#include <ros/ros.h>
#include <lfd_common/action_complete.h>
#include <lfd_common/state.h>
#include <vector>

/*!
 * \def MAX_DATA_POINTS
 * The max data points ROS parameter name
 */
#define MAX_DATA_POINTS "~max_data_points"
/*!
 * \def DEFAULT_MAX_POINTS
 * The default number of max data points to allocate
 */
#define DEFAULT_MAX_POINTS 1024
/*!
 * \def DIST_THRESH_MULT
 * The distance threshold multiplier
 */
#define DIST_THRESH_MULT "~dist_thresh_mult"
/*!
 * \def DEFAULT_DIST_MULT
 * The default distance threshold multiplier
 */
#define DEFAULT_DIST_MULT 1.5
/*!
 * \def ANN_EPSILON
 * The error bound on ANN
 */
#define ANN_EPSILON 0

/*!
 * \struct prediction
 * A prediction contains the classification label, confidence, and decision boundary label for a given classification.
 */
typedef struct
{
  int l;
  double c;
  int db;
} prediction;

/*!
 * \struct conf
 * A confidence threshold value for a given action label and decision boundary pair as well as a counter indicating the number of miss-classified points used in the computation of this threshold.
 */
typedef struct
{
  int l;
  int db;
  int cnt;
  double thresh;
} conf;

/*!
 * \class cba_learner
 * \brief Object used to handle communication between other ROS nodes and CBA.
 *
 * The cba_learner handles communication between the agent, human, and CBA via ROS. ROS nodes and services are created and maintained within this object.
 */
class cba_learner
{
public:
  /*!
   * \brief Creates a cba_learner using optional ROS parameters.
   *
   * Creates a cba_learner object that will allocate most of the necessary resources needed and initialize all ROS topics/services. Optional parameters can be supplied via ROS parameters to set the data set size and distance threshold multiplier.
   */
  cba_learner();

  /*!
   * \brief cba_learner destructor.
   *
   * Frees the resources used by the CBA learner.
   */
  virtual ~cba_learner();

  /*!
   * \brief Iterates once trough the CBA algorithm.
   *
   * Used to iterate once through the CBA algorithm. This loop will update the learner and handle any ROS messages/service requests that must be made.
   */
  void step();

private:
  /*!
   * \brief Used to classify the current state.
   *
   * Query the classifier for the label of the current state. If the communication to the classifier cannot be made, a label of -1 is given with negative infinity confidence.
   *
   * \return the prediction information for the current state
   */
  prediction *classify_state();

  /*!
   * \brief Used to calculate the nearest neighbor to the current state.
   *
   * Calculate and return the nearest neighbor distance to the current state using ANN. If no data points exist in the data set, infinity is returned.
   *
   * \return the nearest neighbor distance to the current state
   */
  double nearest_neighbor();

  /*!
   * \brief Retrieve the confidence threshold for the given label and decision boundary pair.
   *
   * Find the confidence threshold for the given label and decision boundary pair. If no threshold value has been set for this pair yet, infinity is returned (the initial threshold).
   *
   * \param l the label value
   * \param db the decision boundary label value
   * \return the confidence threshold for the given label and decision boundary pair
   */
  double conf_thresh(int l, int db);

  /*!
   * \brief Update the distance and confidence thresholds.
   *
   * Update the distance and confidence thresholds as defined by the CBA algorithm.
   */
  void update_thresholds();

  /*!
   * \brief state_listener topic callback function
   *
   * Copies the given state vector into a global buffer. If this is the first call to this function, several buffer resources are allocated based on the size of the state vector.
   *
   * \param msg the current state vector of the agent
   */
  void state_listener_callback(const lfd_common::state::ConstPtr &msg);

  /*!
   * \brief a_complete service callback function
   *
   * Set the action complete flag to true and autonomous action flag to false. Furthermore, if the action completed was autonomous and a correction was given by the user, the model is updated appropriately.
   *
   * \param req the request for the a_complete service
   * \param resp the response for the a_complete service; this does not contain any information for this service
   * \return returns true once completed
   */
  bool a_complete_callback(lfd_common::action_complete::Request &req, lfd_common::action_complete::Response &resp);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Publisher execute, add_point, change_point; /*!< the execute, add_point, and change_point topics */
  ros::Subscriber state_listener; /*!< the state_listener topic */
  ros::ServiceClient classify, correction, demonstration; /*!< the classify, correction, and demonstration services */
  ros::ServiceServer a_complete; /*!< the a_complete service */

  float *s, *sc; /*!< the current state of CBA and the last confidence state */
  int s_size, max_pts, pts, a; /*!< the length of the state vector, maximum number of data points to allocate, current number of data points, and current action for the agent to execute */
  bool action_complete, autonomous_action; /*!< if the action has been reported by the agent as finished and if the current action was executed autonomously */
  double dist_thresh, dist_mult; /*!< the distance threshold value and distance threshold multiplier */
  std::vector<conf*> conf_thresholds; /*!< confidence threshold values for action label and decision boundary pairs */

  ANNpointArray ann_data; /*!< data points for ANN */
  int32_t *labels; /*!< labels for each entry in the data set */
};

/*!
 * Creates and runs the cba node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
