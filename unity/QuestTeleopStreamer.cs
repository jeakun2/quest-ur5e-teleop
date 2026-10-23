// QuestTeleopStreamer.cs — native Unity (Meta Quest) client for the UR5e bridge.
//
// Drop this on any GameObject in an XR scene (OpenXR / XR Interaction Toolkit,
// e.g. the existing G:\Works\Unity\teleop project, Unity 2022.3 + OpenXR 1.10).
// It reads both controllers + head through UnityEngine.XR.InputDevices, converts
// Unity's left-handed frame to the bridge's right-handed XR frame and streams the
// same JSON the WebXR client sends (docs/PROTOCOL.md) over a plain WebSocket.
//
// Run the bridge with TLS off for this client:
//     python bridge/teleop_bridge.py --source.tls false
// and set serverUrl to ws://<PC-IP>:8765/ws
//
// Conversion Unity (x right, y up, +z forward, left-handed)  ->  XR (x right, y up, -z forward):
//     p' = (x, y, -z)            q' = (-qx, -qy, qz, qw)
using System;
using System.Collections.Generic;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.XR;

public class QuestTeleopStreamer : MonoBehaviour
{
    [Tooltip("ws://<bridge PC ip>:8765/ws  (bridge started with --source.tls false)")]
    public string serverUrl = "ws://192.168.1.20:8765/ws";
    [Range(30, 120)] public int sendHz = 72;
    public TextMesh hudText;            // optional: world-space text for bridge status

    ClientWebSocket ws;
    CancellationTokenSource cts;
    float nextSend;
    readonly StringBuilder sb = new StringBuilder(512);
    volatile string lastStatus = "";
    bool connecting;

    void Start() { _ = ConnectLoop(); }

    void OnDestroy() { cts?.Cancel(); ws?.Dispose(); }

    async Task ConnectLoop()
    {
        cts = new CancellationTokenSource();
        while (!cts.IsCancellationRequested)
        {
            if (ws == null || ws.State != WebSocketState.Open)
            {
                if (!connecting)
                {
                    connecting = true;
                    try
                    {
                        ws?.Dispose();
                        ws = new ClientWebSocket();
                        await ws.ConnectAsync(new Uri(serverUrl), cts.Token);
                        Debug.Log("[teleop] connected " + serverUrl);
                        _ = ReceiveLoop();
                    }
                    catch (Exception e) { Debug.LogWarning("[teleop] connect failed: " + e.Message); }
                    connecting = false;
                }
            }
            await Task.Delay(1000);
        }
    }

    async Task ReceiveLoop()
    {
        var buf = new byte[4096];
        try
        {
            while (ws.State == WebSocketState.Open && !cts.IsCancellationRequested)
            {
                var r = await ws.ReceiveAsync(new ArraySegment<byte>(buf), cts.Token);
                if (r.MessageType == WebSocketMessageType.Close) break;
                var s = Encoding.UTF8.GetString(buf, 0, r.Count);
                int i = s.IndexOf("\"state\":\"", StringComparison.Ordinal);
                if (i >= 0) { int j = s.IndexOf('"', i + 9); if (j > i) lastStatus = s.Substring(i + 9, j - i - 9); }
            }
        }
        catch (Exception) { }
    }

    void Update()
    {
        if (hudText) hudText.text = lastStatus;
        if (Time.unscaledTime < nextSend) return;
        nextSend = Time.unscaledTime + 1f / sendHz;
        if (ws == null || ws.State != WebSocketState.Open) return;

        sb.Clear();
        sb.Append("{\"t\":").Append(Time.unscaledTimeAsDouble.ToString("F4", System.Globalization.CultureInfo.InvariantCulture));
        sb.Append(",\"right\":"); AppendController(XRNode.RightHand);
        sb.Append(",\"left\":"); AppendController(XRNode.LeftHand);
        sb.Append(",\"head\":"); AppendHead();
        sb.Append('}');
        var bytes = Encoding.UTF8.GetBytes(sb.ToString());
        try { _ = ws.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text, true, cts.Token); }
        catch (Exception) { }
    }

    static readonly System.Globalization.CultureInfo Inv = System.Globalization.CultureInfo.InvariantCulture;
    void F(float v) { sb.Append(v.ToString("F5", Inv)); }

    void AppendPose(Vector3 p, Quaternion q)
    {
        sb.Append("\"p\":["); F(p.x); sb.Append(','); F(p.y); sb.Append(','); F(-p.z); sb.Append("],");
        sb.Append("\"q\":["); F(-q.x); sb.Append(','); F(-q.y); sb.Append(','); F(q.z); sb.Append(','); F(q.w); sb.Append(']');
    }

    void AppendController(XRNode node)
    {
        var dev = InputDevices.GetDeviceAtXRNode(node);
        if (!dev.isValid) { sb.Append("null"); return; }
        dev.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 p);
        dev.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion q);
        dev.TryGetFeatureValue(CommonUsages.grip, out float grip);
        dev.TryGetFeatureValue(CommonUsages.trigger, out float trig);
        dev.TryGetFeatureValue(CommonUsages.primaryButton, out bool a);
        dev.TryGetFeatureValue(CommonUsages.secondaryButton, out bool b);
        dev.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 axes);
        dev.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out bool stick);
        dev.TryGetFeatureValue(CommonUsages.isTracked, out bool tracked);
        sb.Append('{'); AppendPose(p, q);
        sb.Append(",\"grip\":"); F(grip); sb.Append(",\"trigger\":"); F(trig);
        sb.Append(",\"a\":").Append(a ? "true" : "false").Append(",\"b\":").Append(b ? "true" : "false");
        sb.Append(",\"stick\":").Append(stick ? "true" : "false");
        // bridge convention = WebXR gamepad: axes[1] negative = stick forward (Unity: positive = forward)
        sb.Append(",\"axes\":["); F(axes.x); sb.Append(','); F(-axes.y); sb.Append(']');
        sb.Append(",\"tracked\":").Append(tracked ? "true" : "false").Append('}');
    }

    void AppendHead()
    {
        var dev = InputDevices.GetDeviceAtXRNode(XRNode.CenterEye);
        if (!dev.isValid) { sb.Append("null"); return; }
        dev.TryGetFeatureValue(CommonUsages.centerEyePosition, out Vector3 p);
        dev.TryGetFeatureValue(CommonUsages.centerEyeRotation, out Quaternion q);
        sb.Append('{'); AppendPose(p, q); sb.Append('}');
    }
}
