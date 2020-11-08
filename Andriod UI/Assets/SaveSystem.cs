using UnityEngine;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;

public static class SaveSystem
{
    public static void SaveData (string ip)
    {
        BinaryFormatter formatter = new BinaryFormatter();
        string path = Application.persistentDataPath + "applicationData.bin";
        FileStream stream = new FileStream(path, FileMode.Create);

        ConnectionData data = new ConnectionData(ip);

        formatter.Serialize(stream, data);
        stream.Close();
    }

    public static ConnectionData LoadData ()
    {
        string path = Application.persistentDataPath + "applicationData.bin";
        if (File.Exists(path))
        {
            BinaryFormatter formatter = new BinaryFormatter();
            FileStream stream = new FileStream(path, FileMode.Open);

            ConnectionData data = formatter.Deserialize(stream) as ConnectionData;
            stream.Close();

            return data;
        }
        else
        {
            Debug.LogError("Save file not found in " + path);
            return null;
        }
    }
}
