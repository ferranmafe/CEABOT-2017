#include "ceabot_free_trial_alg.h"

CeabotFreeTrialAlgorithm::CeabotFreeTrialAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

CeabotFreeTrialAlgorithm::~CeabotFreeTrialAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void CeabotFreeTrialAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// CeabotFreeTrialAlgorithm Public API
