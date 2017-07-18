#include "ceabot_sumo_alg_node.h"

CeabotSumoAlgNode::CeabotSumoAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<CeabotSumoAlgorithm>()
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

CeabotSumoAlgNode::~CeabotSumoAlgNode(void)
{
  // [free dynamic memory]
}

void CeabotSumoAlgNode::mainNodeThread(void)
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

void CeabotSumoAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void CeabotSumoAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<CeabotSumoAlgNode>(argc, argv, "ceabot_sumo_alg_node");
}
