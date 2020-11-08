using UnityEngine;
using System.Collections;
using System.Net;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;
using uPLibrary.Networking.M2Mqtt.Utility;
using uPLibrary.Networking.M2Mqtt.Exceptions;
using System;

using UnityEngine.UI;

public class StartTest : MonoBehaviour {

	public InputField ipInputField;
	public Image ipError;
    public string defaultIp;
	private string msg;

	private MqttClient client;
	// Use this for initialization
	void Start () {
		ConnectToMqtt();
	}

	void client_MqttMsgPublishReceived(object sender, MqttMsgPublishEventArgs e) 
	{ 

		Debug.Log("Received: " + System.Text.Encoding.UTF8.GetString(e.Message));
	} 

	public void SendTestMsg() {
        Debug.Log("sending...");
        client.Publish("hello/world", System.Text.Encoding.UTF8.GetBytes("this is a test"), MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE, true);
        Debug.Log("sent");
	}

	void ConnectToMqtt()
    {
		string address = defaultIp;
		ConnectionData data = SaveSystem.LoadData();
		if (data != null)
		{
			address = data.ip;
		}
		ipInputField.text = address;

		// create client instance _
		try
		{
			client = new MqttClient(IPAddress.Parse(address), 1883, false, null);

			// register to message received 
			client.MqttMsgPublishReceived += client_MqttMsgPublishReceived;

			string clientId = Guid.NewGuid().ToString();
			client.Connect(clientId);

			// subscribe to the topic "/home/temperature" with QoS 2 
			client.Subscribe(new string[] { "hello/world" }, new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });


			ipError.gameObject.SetActive(false);
		}
		catch (Exception e)
		{
			ipError.gameObject.SetActive(true);
			Debug.LogException(e, this);
		}
	}

	public void UpdateConnection()
    {
		string address = ipInputField.text;
		SaveSystem.SaveData(address);
		ConnectToMqtt();
	}

}
