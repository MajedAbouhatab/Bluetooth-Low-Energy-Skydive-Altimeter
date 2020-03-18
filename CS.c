// ----------------------------------------------------------------------------
// Copyright (c) 2018 Semiconductor Components Industries LLC
// (d/b/a "ON Semiconductor").  All rights reserved.
// This software and/or documentation is licensed by ON Semiconductor under
// limited terms and conditions.  The terms and conditions pertaining to the
// software and/or documentation are available at
// http://www.onsemi.com/site/pdf/ONSEMI_T&C.pdf ("ON Semiconductor Standard
// Terms and Conditions of Sale, Section 8 Software") and if applicable the
// software license agreement.  Do not use this software and/or documentation
// unless you have carefully read and you agree to the limited terms and
// conditions.  By using this software and/or documentation, you agree to the
// limited terms and conditions.
// ----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// INCLUDES
//-----------------------------------------------------------------------------

#include "RTE_Components.h"

#include <ctype.h>
#include <ics/CS.h>
#include <ics/CS_Platform.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <BitBanging.h>
#include <math.h>
int InitialAlt = -9999;
int OffsetAlt;
int AGL;
int PreviousAGL = -9999;

#if defined RTE_DEVICE_BDK_OUTPUT_REDIRECTION && CS_LOG_WITH_ANSI_COLORS != 0
#include "ansi_color.h"
#endif

//-----------------------------------------------------------------------------
// DEFINES / CONSTANTS
//-----------------------------------------------------------------------------

#define CSN_SYS_NODE_NAME "SYS"

#define CSN_SYS_AVAIL_BIT ((uint32_t)0x00000000)

//-----------------------------------------------------------------------------
// FORWARD DECLARATIONS
//-----------------------------------------------------------------------------

static int CSN_SYS_RequestHandler(const struct CS_Request_Struct *request,
		char *response);

//-----------------------------------------------------------------------------
// INTERNAL VARIABLES
//-----------------------------------------------------------------------------

static struct CS_Handle_Struct cs;

static char cs_tx_buffer[21];

static char cs_rx_buffer[21];

static char cs_node_response[21];

static struct CS_Node_Struct cs_sys_node = {
CSN_SYS_NODE_NAME,
CSN_SYS_AVAIL_BIT, &CSN_SYS_RequestHandler };

//-----------------------------------------------------------------------------
// FUNCTION DEFINITIONS
//-----------------------------------------------------------------------------

int CalculateAltitude(float P, float T) {
	// Get Altitude from Pressure and Temperature
	return (int) ((pow(P / 101.325, 0.190223) - 1) * (T * 280.4137 + 128897.8));
}

void SendAltitudeToDisplay(void) {
	// Save the response values
	float ResponseP, ResponseT;
	// Send a request in a specific format
	cs.node[1]->request_handler((struct CS_Request_Struct[] ) {
					{ "5", "EV", "P" } }, cs_node_response);
	// Save Pressure value
	ResponseP = atof(&cs_node_response[2]);
	if (ResponseP != 0.00) {
		// Send a request in a specific format
		cs.node[1]->request_handler((struct CS_Request_Struct[] ) { { "6", "EV",
						"T" } }, cs_node_response);
		// Save Temperature value
		ResponseT = atof(&cs_node_response[2]);
		// Right align text on OLED
		char TextBuffer[5];
		// Check if baseline altitude has been set
		if (InitialAlt == -9999) {
			// Set baseline
			InitialAlt = CalculateAltitude(ResponseP, ResponseT);
		} else {
			// Measure from baseline and take into account external Altitude adjustment
			AGL = InitialAlt - CalculateAltitude(ResponseP, ResponseT)
					+ OffsetAlt;
			// Less than 1000 feet
			if (AGL < 1000) {
				// Print into 4 places to right align text on OLED
				snprintf(TextBuffer, sizeof(TextBuffer), "%4d", AGL);
			} else {
				// Less than 10000
				if (AGL < 10000) {
					// Print whole number into two places and one after the decimal
					snprintf(TextBuffer, sizeof(TextBuffer), "%2d.%d",
							AGL / 1000, AGL % 1000 / 100);
				} else {
					// Just use all of OLED
					snprintf(TextBuffer, sizeof(TextBuffer), "%d.%d",
							AGL / 1000, AGL % 1000 / 100);
				}
			}
			// Send to OLED if it has been more than 0.2 seconds since last update
			////if (HAL_Time() - TimeStamp > 200) {
			//sd1306_draw_string(0, 15, TextBuffer, 5, white_pixel);
			// Loop through digits
			for (uint16_t k = 0; k < 4; k++) {
				// Remove ASCII offset
				char digit = TextBuffer[k] - 48;
				// Adjust if decimal point or negative
				if (digit < 0)
					digit = digit + 13;
				// Loop through pages
				for (uint16_t j = 0; j < 8; j++)
					// Columns of each page
					for (uint16_t i = j * 128; i < j * 128 + 32; i++)
						// Copy to OLED buffer in specific order
						if (digit >= 0)
							oled_buffer[i + k * 32] = Image_num_bmp[i
									- (j * 128) + (j * 32) + (digit * 256)];
						else
							// If blank
							oled_buffer[i + k * 32] = 0x00;
			}
			// Send OLED buffer to sd1306
			WriteBufferToDisplay();
			// Send via BLE only if AGL value has changed
			if (PreviousAGL != AGL)
				CS_PlatformWrite(TextBuffer, sizeof(TextBuffer));
			TimeStamp = HAL_Time();
			PreviousAGL = AGL;
			////}
		}
	}
}

int CS_Init() {
	int i, errcode;

	// Initialize structure with default values
	cs.node_cnt = 0;
	for (i = 0; i < CS_MAX_NODE_COUNT; ++i) {
		cs.node[i] = NULL;
	}
	cs.conf_content = NULL;
	cs.conf_content_len = 0;
	cs.conf_page_cnt = 0;

	errcode = CS_PlatformInit(&cs);
	if (errcode != CS_OK) {
		CS_SYS_Error("Platform initialization failed.");
		return errcode;
	}

	CS_SYS_Info("Platform initialized.");

	// Add SYS service node.
	CS_RegisterNode(&cs_sys_node);

	return CS_OK;
}

int CS_RegisterNode(struct CS_Node_Struct *node) {
	if (node == NULL || node->name == NULL || node->request_handler == NULL) {
		CS_SYS_Error("Failed to register node.");
		return CS_ERROR;
	}

	if (cs.node_cnt == CS_MAX_NODE_COUNT) {
		CS_SYS_Error("Reached maximum node count.");
		return CS_ERROR;
	}

	cs.node[cs.node_cnt] = node;
	cs.node_cnt += 1;

	CS_SYS_Info("Registered node '%s'", cs.node[cs.node_cnt - 1]->name);
	return CS_OK;
}

int CS_Loop(int timeout) {
	int errcode, bytes, i;
	uint32_t timestamp;
	struct CS_Request_Struct request;

	// Read available BLE packet
	memset(cs_rx_buffer, 0, 21);
	CS_PlatformRead(cs_rx_buffer, 21, &bytes);
	// New packet available
	if (bytes > 0) {
		// New AGL offset
		OffsetAlt = OffsetAlt + atoi(cs_rx_buffer);
		// Fore update BLE
		PreviousAGL = -9999;
	}
	// We're done here
	return 0;

	// Wait for BLE data with 10 ms timeout.
	errcode = CS_PlatformSleep(timeout);
	if (errcode != CS_OK) {
		return errcode;
	}

	timestamp = CS_PlatformTime();

	// Read available BLE packet.
	memset(cs_rx_buffer, 0, 21);
	errcode = CS_PlatformRead(cs_rx_buffer, 21, &bytes);
	if (errcode != CS_OK || bytes <= 0) {
		CS_SYS_Error("Platform read failed. (errcode=%d)", errcode);
		return CS_ERROR;
	}

#if CS_LOG_WITH_ANSI_COLORS != 0 && defined RTE_DEVICE_BDK_OUTPUT_REDIRECTION
	CS_SYS_Info("Received request packet: '" COLORIZE("%s", CYAN, BOLD) "'",
			cs_rx_buffer);
#else
	CS_SYS_Info("Received request packet: '%s'", cs_rx_buffer);
#endif

	// Parse header information
	request.token = strtok(cs_rx_buffer, "/");
	if (request.token == NULL) {
		CS_SYS_Error("Failed to parse request token.");
		return CS_ERROR;
	}
	if (strlen(request.token) != 1) {
		CS_SYS_Error("Invalid request token length.");
		return CS_ERROR;
	}
	if (request.token[0] < '0' || request.token[0] > '~') {
		CS_SYS_Error("Invalid request token value.");
		return CS_ERROR;
	}

	request.node = strtok(NULL, "/");
	if (request.node == NULL) {
		CS_SYS_Error("Failed to parse request node name.");
		return CS_ERROR;
	}

	request.property = strtok(NULL, "/");
	if (request.property == NULL) {
		CS_SYS_Error("Failed to parse request node property.");
		return CS_ERROR;
	}

	// NULL for read requests
	request.property_value = strtok(NULL, "/");

	// Iterate all available nodes to find a match.
	for (i = 0; i < cs.node_cnt; ++i) {
		if (strcmp(request.node, cs.node[i]->name) == 0) {
			// Matching node was found -> pass request
			errcode = cs.node[i]->request_handler(&request, cs_node_response);
			if (errcode == CS_OK && strlen(cs_node_response) <= 18) // 2b for token + response = 20b
					{
				// Compose response packet from token + node response
				sprintf(cs_tx_buffer, "%s/%s", request.token, cs_node_response);
#if CS_LOG_WITH_ANSI_COLORS != 0 && defined RTE_DEVICE_BDK_OUTPUT_REDIRECTION
				CS_SYS_Info(
						"Composed response packet '" COLORIZE("%s", MAGENTA, BOLD) "'",
						cs_tx_buffer);
#else
				CS_SYS_Info("Composed response packet '%s'", cs_tx_buffer);
#endif

				// Send response to platform
				int res_len = strlen(cs_tx_buffer);
				if (CS_PlatformWrite(cs_tx_buffer, res_len) == CS_OK) {
					timestamp = CS_PlatformTime() - timestamp;
					CS_SYS_Verbose("Request completed in %lu ms.", timestamp);
					return CS_OK;
				} else {
					CS_SYS_Error("Platform send failed. (errcode=%d)", errcode);
					return CS_ERROR;
				}
			} else {
				CS_SYS_Error("Node request processing error. (errcode=%d)",
						errcode);
				sprintf(cs_tx_buffer, "%s/e/UNK_ERROR", request.token);
				errcode = CS_PlatformWrite(cs_tx_buffer, strlen(cs_tx_buffer));
				if (errcode != CS_OK) {
					CS_SYS_Error("Platform send failed. (errcode=%d)", errcode);
					return CS_ERROR;
				}
				// Node failed to process request
				return CS_ERROR;
			}
		}
	}

	CS_SYS_Error("No matching node found for '%s'", request.node);
	sprintf(cs_tx_buffer, "%s/e/UNK_NODE", request.token);
	errcode = CS_PlatformWrite(cs_tx_buffer, strlen(cs_tx_buffer));
	if (errcode != CS_OK) {
		CS_SYS_Error("Platform send failed. (errcode=%d)", errcode);
		return CS_ERROR;
	}

	return CS_ERROR;
}

void CS_SetAppConfig(const char *content) {
	if (content == NULL) {
		return;
	}

	cs.conf_content_len = strlen(content);
	if (cs.conf_content_len == 0) {
		return;
	}

	cs.conf_page_cnt = (cs.conf_content_len / CS_TEXT_PAGE_LEN);
	if (cs.conf_page_cnt * CS_TEXT_PAGE_LEN < cs.conf_content_len) {
		cs.conf_page_cnt += 1;
	}

	cs.conf_content = content;

	CS_SYS_Info("Set app config. %dB long, %d pages.", cs.conf_content_len,
			cs.conf_page_cnt);
}

void CS_Log(enum CS_Log_Level level, const char *module, const char *fmt, ...) {

#if defined RTE_DEVICE_BDK_OUTPUT_REDIRECTION && CS_LOG_WITH_ANSI_COLORS != 0
	static const char *log_level_str[] = { COLORIZE("ERROR", RED, BOLD),
			COLORIZE("WARN", YELLOW, BOLD), COLORIZE("INFO", GREEN), "VERBOSE" };
#else
    static const char* log_level_str[] = {
        "ERROR",
        "WARN",
        "INFO",
        "VERBOSE"
    };
#endif

	if (module != NULL && fmt != NULL) {
		CS_PlatformLogLock();

		CS_PlatformLogPrintf("[CS %s][%s] ", log_level_str[level], module);
		va_list args;
		va_start(args, fmt);
		CS_PlatformLogVprintf(fmt, args);
		va_end(args);
		CS_PlatformLogPrintf("\r\n");

		CS_PlatformLogUnlock();
	}
}

static int CSN_SYS_RequestHandler(const struct CS_Request_Struct *request,
		char *response) {
	// CONF property request (RW)
	if (strcmp(request->property, "CONF") == 0) {
		if (request->property_value == NULL) {
			// Read -> Return page count
			sprintf(response, "i/%d", cs.conf_page_cnt);
			return CS_OK;
		} else {
			if (cs.conf_content == NULL) {
				strcpy(response, "e/NO_CONF");
				return CS_OK;
			}

			int page_id = atoi(request->property_value);
			if (page_id >= 0 && page_id < cs.conf_page_cnt) {
				int content_index = page_id * CS_TEXT_PAGE_LEN;
				sprintf(response, "t/%.*s", CS_TEXT_PAGE_LEN,
						cs.conf_content + content_index);
				return CS_OK;
			} else {
				strcpy(response, "e/INV_INDEX");
				return CS_OK;
			}
		}
	}

	// Check request type. Only R requests below.
	if (request->property_value != NULL) {
		CS_SYS_Error("SYS properties support only read requests.");
		sprintf(response, "e/ACCESS");
		return CS_OK;
	}

	// AVAIL property request
	if (strcmp(request->property, "AVAIL") == 0) {
		int i;
		uint32_t avail_bit_map = 0;

		for (i = 0; i < cs.node_cnt; ++i) {
			avail_bit_map |= cs.node[i]->avail_bit;
		}

		sprintf(response, "h/%lX", avail_bit_map);
		return CS_OK;
	}

	// NODE property request
	if (strcmp(request->property, "NODE") == 0) {
		sprintf(response, "i/%d", cs.node_cnt - 1);
		return CS_OK;
	}

	// NODEx property request
	if (strlen(request->property) > 4
			&& memcmp(request->property, "NODE", 4) == 0) {
		// check if there are only digits after first 4 characters
		char *c = (char*) &request->property[4];
		int valid_number = 1;
		while (*c != '\0') {
			if (isdigit(*c) == 0) {
				valid_number = 0;
				break;
			}
			++c;
		}

		if (valid_number == 1) {
			int node_index = atoi(&request->property[4]);
			if (node_index >= 0 && node_index < (cs.node_cnt - 1)) {
				sprintf(response, "n/%s", cs.node[node_index + 1]->name);
				return CS_OK;
			} else {
				CS_SYS_Error("Out of bound NODEx request.");
				// Invalid property error
			}
		} else {
			// Invalid property error
		}
	}

	CS_SYS_Error("SYS property '%s' does not exist.", request->property);
	sprintf(response, "e/UNK_PROP");
	return CS_OK;
}
