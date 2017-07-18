#include "ceabot_free_trial_alg_node.h"

CeabotFreeTrialAlgNode::CeabotFreeTrialAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<CeabotFreeTrialAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

CeabotFreeTrialAlgNode::~CeabotFreeTrialAlgNode(void)
{
  // [free dynamic memory]
}

void CeabotFreeTrialAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void CeabotFreeTrialAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void CeabotFreeTrialAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<CeabotFreeTrialAlgNode>(argc, argv, "ceabot_free_trial_alg_node");
}
