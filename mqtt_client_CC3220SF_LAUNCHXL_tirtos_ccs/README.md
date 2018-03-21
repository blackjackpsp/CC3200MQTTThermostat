## Example Summary

This example introduce the MQTT Client library API and usage.

## Peripherals Exercised

* The board LEDs are used for status indication. To distinguish similar indications, the user needs to be aware of the executed procedure.   
The following table lists all options.

<table>
  <tr>
    <th>LED indication</th>
    <th>Led Color</th> 
    <th>Interpretation</th>
  </tr>
  <tr>
    <td>Solidly on</td>
    <td>Green</td> 
    <td>Indicate Simplelink is properly up - Every Reset / Initialize</td>
  </tr>
  <tr>
    <td>Blinking</td>
    <td>Red</td> 
    <td>Device is trying to connect to AP - Every Reset / Initialize</td>
  </tr>
  <tr>
    <td>Solidly off</td>
    <td>All</td> 
    <td>Device connected and working - Only after connection</td>
  </tr>
  <tr>
    <td>Toggling (Solidly on/off)</td>
    <td>Red</td> 
    <td>Publish message received in SUBSCRIPTION_TOPIC1</td>
  </tr>
  <tr>
    <td>Toggling (Solidly on/off)</td>
    <td>Yellow</td> 
    <td>Publish message received in SUBSCRIPTION_TOPIC2</td>
  </tr>
  <tr>
    <td>Toggling (Solidly on/off)</td>
    <td>Green</td> 
    <td>Publish message received in SUBSCRIPTION_TOPIC3</td>
  </tr>
</table>

## Example Usage

* Access Point (AP) Configuration
	- AP information is set in 'network\_if.h' file.

The application have one MQTT entity  
MQTT Client Role - Can connect to remote broker 

* Remote Broker Configuration

	- Broker parameters can be configured in Mqtt\_ClientCtx parameter which can be found in 'mqtt\_server\_app.c'
	- The broker parameters are:
		- Connection types and security options
			- IPv4 connection
			- IPv6 connection
			- URL connection
			- Secure connection
			- skip domain name verfication in secure connection
			- skip certificate catalog verfication in secture connection
		- Server Address: URL or IP
    	- Port number of MQTT server
    	- Method to tcp secured socket
    	- Cipher to tcp secured socket
    	- Number of files for secure transfer
    	- The secure Files  

* Secured socket  
	In order to activate the secured example, SECURE\_CLIENT must be defined  ( certificates should be programmed ).

* Client Authentication  
	In order to activate the Client authentication by the server, CLNT\_USR\_PWD must be defined  ( ClientUsername and ClientPassword must be defined ).  
  
* Topics Configuration
	- The topics can be set by changing the definitions in 'mqtt\_client\_app.c' file  
	- The subscription topics can be set in the **SUBSCRIPTION\_TOPICX** definitions
	- The Client is subscribe to the following default topics  
		**"/Broker/To/cc32xx"**  
		**"/cc3200/ToggleLEDCmdL1"**  
		**"/cc3200/ToggleLEDCmdL2"**  
		**"/cc3200/ToggleLEDCmdL3"**  
	- The publish topic and data can be set in the **PUBLISH\_TOPICX** and **PUBLISH\_TOPICX\_DATA** definitions	  
	- The Client publish the following default topic "/cc32xx/ButtonPressEvtSw2" - 
				the topic will be published by pressing SW2 on the board
	
* Build the project and flash it by using the Uniflash tool for cc32xx, or equivalently, run debug session on the IDE of your choice.

* Open a serial port session (e.g. 'HyperTerminal','puTTY', 'Tera Term' etc.) to the appropriate COM port - listed as 'User UART'.  
The COM port can be determined via Device Manager in Windows or via `ls /dev/tty*` in Linux.

	The connection should have the following connection settings:

    	Baud-rate:    115200
	    Data bits:         8
	    Stop bits:         1
	    Parity:         None
	    Flow Control:   None


* Run the example by pressing the reset button or by running debug session through your IDE.  
 `Green LED` turns ON to indicate the Application initialization is complete 

* Once the application has completed it's initialization and the network processor is up,  
  the application banner would be displayed, showing version details:

        ============================================
           MQTT client Example Ver: 1.1.1
        ============================================

         CHIP: 0x30000019
         MAC:  2.0.0.0
         PHY:  2.2.0.5
         NWP:  3.3.99.2
         ROM:  0
         HOST: 2.0.1.17
         MAC address: 04:a3:16:45:89:8e

        ============================================

* At this point `Board_LED0` will blink until the device will connect to the hard coded AP.  
	* In case connection to the hard coded SSID AP fails, user will be requested to fill the SSID of an open AP it wishes to connect to.
  	* If no AP is available or connection failed, the example reset and tries to connect again.
	* Once the connection success all LEDs turn off.

* Special handlings
	- In case the client will disconnect (for any reason) from the remote broker, the MQTT will be restarted.   
	The user can change that behavior by deleting **gResetApplication = true** from *MQTT\_CLIENT\_DISCONNECT\_CB\_EVENT* case in 'Client\_server\_cbs.c' file. 

## Application Design Details

MQTT Client application used to demonstrate the client side of the MQTT protocol.
It does that by offering a semi-automatic application.

The application starts with invoking the 'Mqtt\_IF\_Connect()' command which tries to connect to a specific AP which his SSID is hard coded in Network\_if.h file, in case the connection to the AP fails, the user will be requested to fill the SSID of an open AP to connect to.

After connection success, the application invokes the 'MqttClient\_start' command which set all the parameter that are required to create TCP/TLS/SSL connection to the broker, invokes the 'MQTTClient\_connect(gMqttClient)' command which create the connection to the broker and then 
invokes the 'MQTTClient_subscribe(gMqttClient , subscriptionInfo, SUBSCRIPTION\_TOPIC\_COUNT)' command which subscribe the client topics which are hard coded in 'Mqtt\_client\_app.c' 

Now the client can receive publish messages from the borker.
In this example the topics in the left will toggle the LEDs in the right
     
							"/cc3200/ToggleLEDCmdL1" <-------------> toggle LED0  
							"/cc3200/ToggleLEDCmdL2" <-------------> toggle LED1  
							"/cc3200/ToggleLEDCmdL3" <-------------> toggle LED2   


The user can invoke more commands by pressing the push buttons on the CC32XX launchpad device: 

* When pressing push botton 0 - SW2, The device will publish the message that include the topic and data which is hard coded in 'Mqtt\_client\_app.c' by invoking 'MQTTClient\_publish' command.

* When pressing push botton 1 - SW3, The device will unsbscribe the topic by invoking 'MQTTClient\_unsubscribe' command, disconnect from the broker by invoking 'MQTTClient\_delete' command and software reset the device.   
	
## References

[MQTT Org - MQTT Home page](http://mqtt.org/documentation)  
[MQTT v3.1.1 specification](http://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html)  
[MQTT v3.1 specification](http://www.ibm.com/developerworks/webservices/library/ws-mqtt/index.html)  