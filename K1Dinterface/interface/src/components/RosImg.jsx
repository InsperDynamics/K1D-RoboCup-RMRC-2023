function RosImg({ros, topic, msg, width, height}) {
  var data = null;
  function subWebcam() {
    var webcamTopic = new window.ROSLIB.Topic({
      ros: ros,
      name: topic,
      messageType: msg
    });
    webcamTopic.subscribe((message) => {
      data = message.data;
      webcamTopic.unsubscribe();
    });
  }
  subWebcam();
  return (
    <img src={data ? `data:image/jpeg;base64,${data}`: `https://via.placeholder.com/${width}x${height}`} width={width} height={height} alt="camera"/>
  );
}

export default RosImg;
