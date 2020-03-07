#include "ceabot_stairs_alg.h"

CeabotStairsAlgorithm::CeabotStairsAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

CeabotStairsAlgorithm::~CeabotStairsAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void CeabotStairsAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// CeabotStairsAlgorithm Public API
