#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "wbr/firmware/hardware_interface.h"
#include "wbr/firmware/control_thread.h"

LOG_MODULE_REGISTER(wbr_app, LOG_LEVEL_INF);

int main() {
  if (!wbr::firmware::InitializeHardware()) {
    wbr::firmware::DisableAllMotors();
    LOG_ERR("hardware port is unavailable; motors remain disabled");
  } else {
    wbr::firmware::DisableAllMotors();
    wbr::firmware::StartControlThread();
    LOG_INF("hardware initialized; 1 kHz control thread started");
  }

  while (true) {
    k_sleep(K_SECONDS(1));
  }
  return 0;
}
