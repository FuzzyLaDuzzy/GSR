const socket = io();

socket.on('connect', () => {
    console.log('Connected to server');
});

socket.on('sensor_update', (data) => {
    for (const sensorId in data) {
        const sensor = data[sensorId];
        document.getElementById(`sensor-${sensorId}-value`).textContent = sensor.value;
        document.getElementById(`sensor-${sensorId}-last-sampling-time`).textContent = new Date(sensor.last_sampling_time * 1000).toLocaleString();
        document.getElementById(`sensor-${sensorId}-sampling-rate`).textContent = sensor.sampling_rate;
    }
});
