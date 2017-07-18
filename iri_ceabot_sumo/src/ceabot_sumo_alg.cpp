#include "ceabot_sumo_alg.h"

CeabotSumoAlgorithm::CeabotSumoAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

CeabotSumoAlgorithm::~CeabotSumoAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void CeabotSumoAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// CeabotSumoAlgorithm Public API
