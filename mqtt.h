#ifndef MQTT_H
#define MQTT_H

/**
 * Initialize the MQTT client and connect to the broker.
 */
void mqtt_init(void);

/**
 * Publish a message to the specified MQTT topic.
 * @param topic The MQTT topic to publish to.
 * @param data The message payload to publish.
 */
void mqtt_publish(const char *topic, const char *data);

#endif // MQTT_H