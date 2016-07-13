/*
	Copyright (C) 2013 - 2014 CurlyMo

	This file is part of pilight.

	pilight is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software
	Foundation, either version 3 of the License, or (at your option) any later
	version.

	pilight is distributed in the hope that it will be useful, but WITHOUT ANY
	WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
	A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with pilight. If not, see	<http://www.gnu.org/licenses/>
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../core/pilight.h"
#include "../core/common.h"
#include "../core/dso.h"
#include "../core/log.h"
#include "../core/json.h"
#include "../core/irq.h"
#include "../core/time.h"
#include "../config/hardware.h"
#include "../../wiringx/wiringX.h"
#include "433gpio.h"

static unsigned short gpio433HwInit(void);
static unsigned short gpio433HwDeinit(void);
static int gpio433Send(int *code, int rawlen, int repeats);
static int gpio433Receive(void);
static unsigned short gpio433Settings(JsonNode *json);
void gpio433Init(void);

static void* gpio433SendHandler(void *arg);
static int gpio433SendQueue(int *code, int rawlen, int repeats);
static int gpio433SendCSMACD(int *code, int rawlen, int repeats);
static void gpio433ReportRawCode(int *code, int rawlen, unsigned long duration, struct timespec* endts);
static void gpio433ReportCode(int *code, int rawlen, unsigned long duration, struct timespec* endts);

typedef enum gpio433_mediaaccesscontrol_t {
	GPIO_433_MACNONE = 0,
	GPIO_433_MACCSMA,
	GPIO_433_MACCSMACD
} gpio433_mediaaccesscontrol_t;

typedef struct gpio433_sendqueue_t {
	int* code;
	int rawlen;
	int repeats;
	int retry;
	struct gpio433_sendqueue_t* next;
} gpio433_sendqueue_t;

typedef struct gpio433_csmadata_t {
	int last_rawlen;
	struct timespec send_lockedts;
	pthread_mutex_t send_lock;
	pthread_cond_t send_signal;
	pthread_condattr_t send_signal_attr;
	struct sendqueue_t *sendqueue_head;
	struct sendqueue_t *sendqueue_tail;
	pthread_mutex_t sendqueue_lock;
	pthread_cond_t sendqueue_signal;
	pthread_t sendthread;
	bool sendthread_stop;
	int (*sendimpl)(int *code, int rawlen, int repeats);
} gpio433_csmadata_t;

static int gpio_433_in = 0;
static int gpio_433_out = 0;
static enum gpio433_mediaaccesscontrol_t gpio_433_mac;
static unsigned int gpio_433_max_retries;
static unsigned int gpio_433_wait_n_frames;
static unsigned long gpio_433_collision_waitmin;
static unsigned long gpio_433_collision_waitunit;
static unsigned int gpio_433_collision_waitmaxunits;

static struct timespec gpio_433_receive_last;
static struct gpio433_csmadata_t* gpio_433_csmadata;

static unsigned long gpio433CalcLockDuration(int *code, int rawlen, unsigned long duration) {
	if(duration == 0) {
		int i;
		for(i=0;i<rawlen;i++) {
			duration += code[i];
		}
	}

	return duration;
}

static void gpio433LockSend(unsigned long duration, struct timespec* from) {
	struct timespec tmp;
	if(from == NULL) {
		clock_gettime(CLOCK_MONOTONIC, &tmp);
	} else {
		tmp = *from;
	}

	time_add_us(&tmp, duration);
	if(time_cmp(&tmp, &csmadata->send_lockedts) > 0) {
		csmadata->send_lockedts = tmp;
	}
}

static unsigned short gpio433HwInit(void) {
	if(wiringXSupported() == 0) {
		if(wiringXSetup() == -1) {
			return EXIT_FAILURE;
		}
		if(gpio_433_out >= 0) {
			if(wiringXValidGPIO(gpio_433_out) != 0) {
				logprintf(LOG_ERR, "invalid sender pin: %d", gpio_433_out);
				return EXIT_FAILURE;
			}
			pinMode(gpio_433_out, OUTPUT);
		}
		if(gpio_433_in >= 0) {
			if(wiringXValidGPIO(gpio_433_in) != 0) {
				logprintf(LOG_ERR, "invalid receiver pin: %d", gpio_433_in);
				return EXIT_FAILURE;
			}
			if(wiringXISR(gpio_433_in, INT_EDGE_BOTH) < 0) {
				logprintf(LOG_ERR, "unable to register interrupt for pin %d", gpio_433_in);
				return EXIT_SUCCESS;
			}

			switch(gpio_433_mac) {
			case GPIO_433_MACNONE:
				break;
			case GPIO_433_MACCSMACD:
				/* no break */
			case GPIO_433_MACCSMA:
				gpio433->sendOOK = &gpio433SendQueue;
				gpio433->reportCode = &gpio433ReportCode;

				gpio_433_csmadata = (struct csmadata_t*) malloc(sizeof(struct csmadata_t));
				if(gpio_433_csmadata == NULL) {
					OUT_OF_MEMORY;
				}
				gpio_433_csmadata->last_rawlen = 0;
				gpio_433_csmadata->send_lockedts.tv_sec = 0;
				gpio_433_csmadata->send_lockedts.tv_nsec = 0;
				pthread_condattr_init(&gpio_433_csmadata->send_signal_attr);
				pthread_condattr_setclock(&gpio_433_csmadata->send_signal_attr, CLOCK_MONOTONIC);
				pthread_mutex_init(&gpio_433_csmadata->send_lock, NULL);
				pthread_cond_init(&gpio_433_csmadata->send_signal, &gpio_433_csmadata->send_signal_attr);
				gpio_433_csmadata->sendqueue_head = gpio_433_csmadata->sendqueue_tail = NULL;
				pthread_mutex_init(&gpio_433_csmadata->sendqueue_lock, NULL);
				pthread_cond_init(&gpio_433_csmadata->sendqueue_signal, NULL);
				gpio_433_csmadata->sendimpl = (gpio_433_mac == MACCSMA) ? &gpio433Send : &gpio433SendCSMACD;
				gpio_433_csmadata->sendthread_stop = false;

				if(pthread_create(&gpio_433_csmadata->sendthread, NULL, gpio433SendHandler, NULL)) {
					logprintf(LOG_ERR, "not able to create send thread");
					return EXIT_FAILURE;
				}
				break;
			default:
				logprintf(LOG_ERR, "invalid mac: %d", mac);
				return EXIT_FAILURE;
			}
		}
		return EXIT_SUCCESS;
	} else {
		logprintf(LOG_ERR, "the 433gpio module is not supported on this hardware", gpio_433_in);
		return EXIT_FAILURE;
	}
}

static unsigned short gpio433HwDeinit(void) {
	switch(mac) {
	case GPIO_433_MACCSMACD:
		/* no break */
	case GPIO_433_MACCSMA:
		gpio_433_csmadata->sendthread_stop = true;
		pthread_mutex_lock(&gpio_433_csmadata->sendqueue_lock);
		pthread_cond_signal(&gpio_433_csmadata->sendqueue_signal);
		pthread_mutex_unlock(&gpio_433_csmadata->sendqueue_lock);
		pthread_mutex_lock(&gpio_433_csmadata->send_lock);
		pthread_cond_signal(&gpio_433_csmadata->send_signal);
		pthread_mutex_unlock(&gpio_433_csmadata->send_lock);

		pthread_join(gpio_433_csmadata->sendthread, NULL);

		while (gpio_433_csmadata->sendqueue_head != NULL) {
			struct sendqueue_t *entry = sendqueue_pop_front();
			free(entry);
		}
		free(gpio_433_csmadata);
		break;
	case GPIO_433_MACNONE:
		break;
	default:
		logprintf(LOG_ERR, "invalid mac: %d", mac);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

static int gpio433Send(int *code, int rawlen, int repeats) {
	int r = 0, x = 0;
	if(gpio_433_out >= 0) {
		for(r=0;r<repeats;r++) {
			for(x=0;x<rawlen;x+=2) {
				digitalWrite(gpio_433_out, 1);
				usleep((__useconds_t)code[x]);
				digitalWrite(gpio_433_out, 0);
				if(x+1 < rawlen) {
					usleep((__useconds_t)code[x+1]);
				}
			}
		}
		digitalWrite(gpio_433_out, 0);
	} else {
		sleep(1);
	}
	return EXIT_SUCCESS;
}

static int gpio433Receive(void) {
	if(gpio_433_in >= 0) {
		return irq_read(gpio_433_in);
	} else {
		sleep(1);
		return 0;
	}
}

static void *gpio433SendHandler(void* arg) {
	struct sendqueue_t *entry = NULL;

	while(1) {
		pthread_mutex_lock(&gpio_433_csmadata->sendqueue_lock);
		entry = sendqueue_pop_front();
		if(entry == NULL) {
			pthread_cond_wait(&gpio_433_csmadata->sendqueue_signal, &gpio_433_csmadata->sendqueue_lock);
			if(gpio_433_csmadata->sendthread_stop) {
				pthread_mutex_unlock(&gpio_433_csmadata->sendqueue_lock);
				goto exit;
			}

			pthread_mutex_unlock(&gpio_433_csmadata->sendqueue_lock);
			continue;
		}
		pthread_mutex_unlock(&gpio_433_csmadata->sendqueue_lock);

		pthread_mutex_lock(&gpio_433_csmadata->send_lock);
		while(1) {
			struct timespec tmp;
			clock_gettime(CLOCK_MONOTONIC, &tmp);
			if(time_cmp(&gpio_433_csmadata->send_lockedts, &tmp) <= 0) {
				break;
			}

			pthread_cond_timedwait(&gpio_433_csmadata->send_signal, &gpio_433_csmadata->send_lock, &gpio_433_csmadata->send_lockedts);
			if(gpio_433_csmadata->sendthread_stop) {
				pthread_mutex_unlock(&gpio_433_csmadata->send_lock);
				goto exit;
			}
		}
		int result = gpio_433_csmadata->sendimpl(entry->code, entry->rawlen, entry->repeats);
		pthread_mutex_unlock(&gpio_433_csmadata->send_lock);

		if(result == EXIT_FAILURE) {
			logprintf(LOG_DEBUG, "send failed (retry %d/%d)", entry->retry, max_retries);
			entry->retry++;
			if(entry->retry < max_retries) {
				pthread_mutex_lock(&gpio_433_csmadata->sendqueue_lock);
				sendqueue_push_front(entry);
				pthread_mutex_unlock(&gpio_433_csmadata->sendqueue_lock);
			} else {
				logprintf(LOG_DEBUG, "skip code");
				free(entry);
				entry = NULL;
			}
		} else {
			free(entry);
			entry = NULL;
		}
	}

exit:
	if (entry != NULL) {
		free(entry);
	}
	return NULL;
}

static int gpio433SendQueue(int *code, int rawlen, int repeats) {
	pthread_mutex_lock(&gpio_433_csmadata->sendqueue_lock);
	bool notify = gpio_433_csmadata->sendqueue_head == NULL;
	struct sendqueue_t *entry = (struct sendqueue_t*) malloc(sizeof(struct sendqueue_t));
	if(entry == NULL) {
		OUT_OF_MEMORY;
	}
	entry->code = (int *) malloc(sizeof(int)*rawlen);
	if(entry->code == NULL) {
		OUT_OF_MEMORY;
	}
	memcpy(entry->code, code, sizeof(int)*rawlen);
	entry->rawlen = rawlen;
	entry->repeats = repeats;
	entry->retry = 0;
	sendqueue_push_back(entry);
	if(notify) {
		pthread_cond_signal(&gpio_433_csmadata->sendqueue_signal);
	}
	pthread_mutex_unlock(&gpio_433_csmadata->sendqueue_lock);
	return EXIT_SUCCESS;
}

static int gpio433SendCSMACD(int *code, int rawlen, int repeats) {
	struct timespec pulse_sent;
	struct timespec now;
	int repeat;
	int transmit;
	int receive;
	int first;
	int last_value;

	for(repeat=0;repeat<repeats;repeat++) {
		transmit = 0;
		receive = 0;
		first = 1;
		last_value = digitalRead(gpio_433_in);

		for(transmit=0;transmit<rawlen;transmit++) {
			int c = !(transmit&1);
			digitalWrite(gpio_433_out, c);

			clock_gettime(CLOCK_MONOTONIC, &pulse_sent);
			time_add_us(&pulse_sent, code[transmit]);

			while(true) {
				clock_gettime(CLOCK_MONOTONIC, &now);
				if(digitalRead(gpio_433_in) != last_value) {
					if(first == 0) {
						if(receive > transmit) {
							goto collision;
						}
						receive++;
					} else {
						first = 0;
					}
					last_value = !last_value;
				}
				if(time_cmp(&now, &pulse_sent) > 0) {
					break;
				}
			}
		}

		digitalWrite(gpio_433_out, 0);
	}

	return EXIT_SUCCESS;

collision:
	digitalWrite(gpio_433_out, 0);

	unsigned long duration = gpio_433_collision_waitmin + gpio_433_collision_waitunit * (rand() % gpio_433_collision_waitmaxunits);
	gpio433LockSend(duration, NULL);
	logprintf(LOG_DEBUG, "csma/cd: lock - collision");

	return EXIT_FAILURE;
}

static void gpio433ReportRawCode(int *code, int rawlen, unsigned long duration, struct timespec* endts) {
	duration = gpio433LockSend(code, rawlen, duration);
	pthread_mutex_lock(&gpio_433_csmadata->send_lock);
	if (gpio_433_csmadata->last_rawlen > 0 && rawlen == gpio_433_csmadata->last_rawlen) {
		unsigned long lock_duration = duration * gpio_433_wait_n_frames;
		gpio433LockSend(lock_duration, endts);
		logprintf(LOG_DEBUG, "csma: raw code deteced (%d times, %lu us)", rawlen, duration);
	}
	gpio_433_csmadata->last_rawlen = rawlen;
	pthread_mutex_unlock(&gpio_433_csmadata->send_lock);
}

static void gpio433ReportCode(int *code, int rawlen, unsigned long duration, struct timespec* endts) {
	duration = gpio433LockSend(code, rawlen, duration);
	pthread_mutex_lock(&gpio_433_csmadata->send_lock);
	unsigned long lock_duration = duration * gpio_433_wait_n_frames;
	gpio433LockSend(lock_duration, endts);
	logprintf(LOG_DEBUG, "csma: code deteced (%d times, %lu us)", rawlen, duration);
	pthread_mutex_unlock(&gpio_433_csmadata->send_lock);
}

static unsigned short gpio433Settings(JsonNode *json) {
	if(strcmp(json->key, "receiver") == 0) {
		if(json->tag == JSON_NUMBER) {
			gpio_433_in = (int)json->number_;
		} else {
			return EXIT_FAILURE;
		}
	} else if(strcmp(json->key, "sender") == 0) {
		if(json->tag == JSON_NUMBER) {
			gpio_433_out = (int)json->number_;
		} else {
			return EXIT_FAILURE;
		}
	} else if(strcmp(json->key, "media-access-control") == 0) {
		if(json->tag == JSON_STRING) {
			if(strcmp(json->string_, "none") == 0) {
				gpio_433_mac = GPIO_433_MACNONE;
			} else if(strcmp(json->string_, "csma") == 0) {
				gpio_433_mac = GPIO_433_MACCSMA;
			} else if(strcmp(json->string_, "csma/cd") == 0) {
				gpio_433_mac = GPIO_433_MACCSMACD;
			} else {
				logprintf(LOG_ERR, "invalid mac: %s", json->string_);
				return EXIT_FAILURE;
			}
		} else {
			return EXIT_FAILURE;
		}
	} else if(strcmp(json->key, "csmacd-wait-min") == 0) {
		if(json->tag == JSON_NUMBER) {
			gpio_433_collision_waitmin = (int)json->number_;
		} else {
			return EXIT_FAILURE;
		}
	} else if(strcmp(json->key, "csmacd-wait-unit") == 0) {
		if(json->tag == JSON_NUMBER) {
			gpio_433_collision_waitunit = (int)json->number_;
		} else {
			return EXIT_FAILURE;
		}
	} else if(strcmp(json->key, "csmacd-wait-max-units") == 0) {
		if(json->tag == JSON_NUMBER) {
			gpio_433_collision_waitmaxunits = (int)json->number_;
		} else {
			return EXIT_FAILURE;
		}
	} else if(strcmp(json->key, "csma-wait-n-frames") == 0) {
		if(json->tag == JSON_NUMBER) {
			gpio_433_wait_n_frames = (int)json->number_;
		} else {
			return EXIT_FAILURE;
		}
	}

	return EXIT_SUCCESS;
}

#if !defined(MODULE) && !defined(_WIN32)
__attribute__((weak))
#endif
void gpio433Init(void) {
	hardware_register(&gpio433);
	hardware_set_id(gpio433, "433gpio");

	options_add(&gpio433->options, 'r', "receiver", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9-]+$");
	options_add(&gpio433->options, 's', "sender", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9-]+$");

	options_add(&gpio433->options, 'm', "media-access-control", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_STRING, NULL, NULL);
	options_add(&gpio433->options, 'a', "csma-wait-n-frames", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9-]+$");
	options_add(&gpio433->options, 'b', "csmacd-wait-min", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9-]+$");
	options_add(&gpio433->options, 'c', "csmacd-wait-unit", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9-]+$");
	options_add(&gpio433->options, 'd', "csmacd-wait-max-units", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9-]+$");

	gpio433->minrawlen = 1000;
	gpio433->maxrawlen = 0;
	gpio433->mingaplen = 5100;
	gpio433->maxgaplen = 10000;

	gpio_433_mac = MACNONE;
	gpio_433_max_retries = 5;
	gpio_433_wait_n_frames = 5;
	gpio_433_collision_waitmin = 200000;
	gpio_433_collision_waitunit = 50000;
	gpio_433_collision_waitmaxunits = 5;
	gpio_433_csmadata = NULL;

	gpio433->hwtype=RF433;
	gpio433->comtype=COMOOK;
	gpio433->init=&gpio433HwInit;
	gpio433->deinit=&gpio433HwDeinit;
	gpio433->sendOOK=&gpio433Send;
	gpio433->receiveOOK=&gpio433Receive;
	gpio433->settings=&gpio433Settings;
}

#if defined(MODULE) && !defined(_WIN32)
void compatibility(struct module_t *module) {
	module->name = "433gpio";
	module->version = "1.3";
	module->reqversion = "7.0";
	module->reqcommit = "10";
}

void init(void) {
	gpio433Init();
}
#endif
