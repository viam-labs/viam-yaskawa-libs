#ifndef VERSION_H
#define VERSION_H
#include "protocol.h"

void comms_get_version_info(version_info_payload_t *info);
const char *comms_version_string(void);
const char *comms_git_commit(void);
#endif
