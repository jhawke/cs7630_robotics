/*!
 * \file cba.cpp
 * \brief Confidence-Based Autonomy (CBA) implementation for ROS
 *
 * cba provides and implementation of the Learning from Demonstration algorithm Confidence-Based Autonomy (CBA) built for ROS.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date December 19, 2011
 */

#include <ANN/ANN.h>
#include <cba/cba.h>
#include <lfd_common/action_complete.h>
#include <lfd_common/classification_point.h>
#include <lfd_common/conf_classification.h>
#include <lfd_common/demonstration.h>
#include <lfd_common/state.h>
#include <limits>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <vector>

using namespace std;

cba_learner::cba_learner()
{
  // add subscriptions and services
  execute = node.advertise<std_msgs::Int32> ("execute", 1);
  add_point = node.advertise<lfd_common::classification_point> ("add_point", -1);
  change_point = node.advertise<lfd_common::classification_point> ("change_point", -1);
  state_listener = node.subscribe<lfd_common::state> ("state_listener", 1, &cba_learner::state_listener_callback, this);
  a_complete = node.advertiseService("a_complete", &cba_learner::a_complete_callback, this);
  classify = node.serviceClient<lfd_common::conf_classification> ("classify");
  correction = node.serviceClient<lfd_common::demonstration> ("correction");
  demonstration = node.serviceClient<lfd_common::demonstration> ("demonstration");

  // check for the maximum number of data points to allocate
  ros::param::param<int>(MAX_DATA_POINTS, max_pts, DEFAULT_MAX_POINTS);
  // check for the distance threshold multiplier
  ros::param::param<double>(DIST_THRESH_MULT, dist_mult, DEFAULT_DIST_MULT);

  // initial CBA values
  labels = NULL;
  s = NULL;
  sc = NULL;
  a = -1;
  s_size = -1;
  pts = 0;
  action_complete = true;
  autonomous_action = false;
  dist_thresh = 0;

  ROS_INFO("CBA Learner Initialized");
}

cba_learner::~cba_learner()
{
  // clear the state vectors if they exists
  if (s != NULL)
    free(s);
  if (sc != NULL)
    free(sc);
  // check the labels set
  if (labels != NULL)
    free(labels);
  // cleanup ANN
  if (pts > 0)
    annDeallocPts(ann_data);
  annClose();
}

void cba_learner::step()
{
  // check if the state exists
  if (s == NULL)
    return;

  // check if the agent has reported their action finished
  if (action_complete)
  {
    // request a prediction from the classifier
    prediction *p = classify_state();

    // calculate the nearest neighbor distance
    double d = nearest_neighbor();

    // check against the thresholds
    if (p->c > conf_thresh(p->l, p->db) && d < dist_thresh)
    {
      // report the action to be executed
      std_msgs::Int32 a;
      a.data = p->l;
      execute.publish(a);
      action_complete = false;
      autonomous_action = true;
      // save a copy of the current state
      memcpy(sc, s, sizeof(float) * s_size);
    }
    else
    {
      // request a demonstration
      if (!demonstration.exists())
        ROS_WARN("Could not connect to 'demonstration' service.");
      else
      {
        // create the service request
        lfd_common::demonstration dem;
        for (int i = 0; i < s_size; i++)
          dem.request.s.state_vector.push_back(s[i]);
        // send the service request
        demonstration.call(dem);

        // check if the user provided a demonstration
        if (dem.response.valid)
        {
          // check if we already have this data point in our set
          bool duplicate;
          for (int i = 0; i < pts; i++)
          {
            for (int j = 0; j < s_size; j++)
              if (ann_data[i][j] == s[j])
                duplicate = true;
              else
              {
                duplicate = false;
                break;
              }
            if (duplicate)
            {
              // check if the label was changed
              if (labels[i] != dem.response.a)
              {
                // change the label
                labels[i] = dem.response.a;
                //update the point in the classifier
                lfd_common::classification_point cp;
                for (int i = 0; i < s_size; i++)
                  cp.s.state_vector.push_back(s[i]);
                cp.l = dem.response.a;
                change_point.publish(cp);

                // update the thresholds
                update_thresholds();
              }
              break;
            }
          }

          // new data point
          if (!duplicate)
          {
            // update the data set
            if (pts == max_pts)
              ROS_WARN("Too many data points -- latest point ignored.");
            else
            {
              // add the data to the ANN set
              for (int i = 0; i < s_size; i++)
                ann_data[pts][i] = s[i];
              // set the label
              labels[pts] = dem.response.a;
              pts++;

              // send the point to the classifier
              lfd_common::classification_point cp;
              for (int i = 0; i < s_size; i++)
                cp.s.state_vector.push_back(s[i]);
              cp.l = dem.response.a;
              add_point.publish(cp);

              // update the thresholds
              update_thresholds();
            }
          }

          // report the action to be executed
          std_msgs::Int32 a;
          a.data = dem.response.a;
          execute.publish(a);
          action_complete = false;
          autonomous_action = false;
        }
      }
    }
    // cleanup
    free(p);
  }
}

prediction *cba_learner::classify_state()
{
  // allocate the prediction
  prediction *p = (prediction *)malloc(sizeof(prediction));

  // check for the classifier
  if (classify.exists())
  {
    // create the service request
    lfd_common::conf_classification cc;
    for (int i = 0; i < s_size; i++)
      cc.request.s.state_vector.push_back(s[i]);

    // send the service request
    classify.call(cc);

    // fill in the struct
    p->c = cc.response.c;
    p->l = cc.response.l;
    p->db = cc.response.db;
  }
  else
  {
    ROS_WARN("Could not connect to 'classify' service.");
    // fill the prediction with negative infinity confidence
    p->c = -numeric_limits<float>::infinity();
    p->db = -1;
    p->l = -1;
  }

  return p;
}

double cba_learner::nearest_neighbor()
{
  // check if we have any data points yet
  if (pts == 0)
    return numeric_limits<float>::infinity();
  else
  {
    // calculate NN using ANN
    ANNidxArray index = new ANNidx[1];
    ANNdistArray dists = new ANNdist[1];

    // create the search structure
    ANNkd_tree *kd_tree = new ANNkd_tree(ann_data, pts, s_size);

    // create the data point
    ANNpoint pt = annAllocPt(s_size);
    for (int i = 0; i < s_size; i++)
      pt[i] = s[i];

    // calculate nearest neighbor
    kd_tree->annkSearch(pt, 1, index, dists, ANN_EPSILON);
    // unsquare the distance
    double d = sqrt(dists[0]);

    // cleanup
    annDeallocPt(pt);
    delete kd_tree;

    return d;
  }
}

double cba_learner::conf_thresh(int l, int db)
{
  // check the thresholds for the given action label and decision boundary pair
  for (uint i = 0; i < conf_thresholds.size(); i++)
    if (conf_thresholds.at(i)->l == l && conf_thresholds.at(i)->db == db)
      return conf_thresholds.at(i)->thresh;

  // no threshold found for the given pair -- we return infinity
  return numeric_limits<float>::infinity();
}

void cba_learner::update_thresholds()
{
  // check if we have at least two points
  if (pts < 2)
    return;

  // clear the old confidence thresholds
  for (uint i = 0; i < conf_thresholds.size(); i++)
    free(conf_thresholds.at(i));
  conf_thresholds.clear();

  // calculate NN using ANN
  ANNidxArray index = new ANNidx[2];
  ANNdistArray dists = new ANNdist[2];
  // create the search structure
  ANNkd_tree *kd_tree = new ANNkd_tree(ann_data, pts, s_size);

  // go through each point and calculate its nearest neighbor and its confidence
  double total_dist = 0;
  for (int i = 0; i < pts; i++)
  {
    // calculate second nearest neighbor (first is itself)
    kd_tree->annkSearch(ann_data[i], 2, index, dists, ANN_EPSILON);
    // add the unsquared distance
    total_dist += sqrt(dists[1]);

    // check for the classifier
    if (classify.exists())
    {
      // create the service request
      lfd_common::conf_classification cc;
      for (int j = 0; j < s_size; j++)
        cc.request.s.state_vector.push_back(ann_data[i][j]);
      // send the service request
      classify.call(cc);

      // find this confidence threshold value
      conf *c = NULL;
      for (uint j = 0; j < conf_thresholds.size() && c == NULL; j++)
        if (conf_thresholds.at(j)->l == cc.response.l && conf_thresholds.at(j)->db == cc.response.db)
          c = conf_thresholds.at(j);
      // if none was found we create a new one
      if (c == NULL)
      {
        c = (conf *)malloc(sizeof(conf));
        c->l = cc.response.l;
        c->db = cc.response.db;
        c->thresh = 0;
        c->cnt = 0;
        conf_thresholds.push_back(c);
      }

      // check if the prediction was correct
      if (cc.response.l != labels[i])
      {
        // add to the average
        c->thresh += cc.response.c;
        c->cnt++;
      }
    }
    else
      ROS_WARN("Could not connect to 'classify' service.");
  }

  // set the distance threshold
  dist_thresh = (total_dist / pts) * dist_mult;

  // set the confidence thresholds
  for (uint i = 0; i < conf_thresholds.size(); i++)
    conf_thresholds.at(i)->thresh /= conf_thresholds.at(i)->cnt;

  // cleanup
  delete kd_tree;
}

void cba_learner::state_listener_callback(const lfd_common::state::ConstPtr &msg)
{
  // set the state size if it is not yet set
  if (s_size == -1)
  {
    s_size = msg->state_vector.size();
    // allocate space for ANN
    ann_data = annAllocPts(max_pts, s_size);
    // allocate space for the states and data sets
    s = (float *)malloc(sizeof(float) * s_size);
    sc = (float *)malloc(sizeof(float) * s_size);
    labels = (int *)malloc(sizeof(int32_t) * max_pts);
  }

  // check if the state sizes match
  if (s_size != (int)msg->state_vector.size())
    ROS_WARN("WARNING: State sizes do not match -- Ignoring current state.");

  // copy the state vector
  for (int i = 0; i < s_size; i++)
    s[i] = msg->state_vector[i];
}

bool cba_learner::a_complete_callback(lfd_common::action_complete::Request &req,
                                      lfd_common::action_complete::Response &resp)
{
  // set the action complete value
  action_complete = true;

  // check if a correction was given for an autonomous action
  if (autonomous_action && req.valid_correction)
  {
    // find the data point
    bool duplicate;
    for (int i = 0; i < pts; i++)
    {
      for (int j = 0; j < s_size; j++)
        if (ann_data[i][j] == sc[j])
          duplicate = true;
        else
        {
          duplicate = false;
          break;
        }
      if (duplicate)
      {
        // check if the label was changed
        if (labels[i] != req.a)
        {
          // change the label
          labels[i] = req.a;
          //update the point in the classifier
          lfd_common::classification_point cp;
          for (int i = 0; i < s_size; i++)
            cp.s.state_vector.push_back(s[i]);
          cp.l = req.a;
          change_point.publish(cp);

          // update the thresholds
          update_thresholds();
        }
      }
    }

    // new data point
    if (!duplicate)
    {
      // update the data set
      if (pts == max_pts)
        ROS_WARN("Too many data points -- latest point ignored.");
      else
      {
        // add the data to the ANN set
        for (int i = 0; i < s_size; i++)
          ann_data[pts][i] = s[i];
        // set the label
        labels[pts] = req.a;
        pts++;

        // send the point to the classifier
        lfd_common::classification_point cp;
        for (int i = 0; i < s_size; i++)
          cp.s.state_vector.push_back(s[i]);
        cp.l = req.a;
        add_point.publish(cp);

        // update the thresholds
        update_thresholds();
      }
    }
  }

  // if the action is complete, it cannot be autonomous anymore
  autonomous_action = false;
  return true;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "cba");

  // initialize the CBA learner
  cba_learner cba;

  // continue until a ctrl-c has occurred
  while (ros::ok())
  {
    ros::spinOnce();
    // step through the algorithm once
    cba.step();
  }

  return EXIT_SUCCESS;
}
