#include "ceabot_stairs_alg_node.h"

CeabotStairsAlgNode::CeabotStairsAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<CeabotStairsAlgorithm>()
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

CeabotStairsAlgNode::~CeabotStairsAlgNode(void)
{
  // [free dynamic memory]
}

void CeabotStairsAlgNode::mainNodeThread(void)
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

void CeabotStairsAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void CeabotStairsAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<CeabotStairsAlgNode>(argc, argv, "ceabot_stairs_alg_node");
}
