class RovisSensor(object):
    def setup_sensor(self, sensor_transform):
        pass

    def process_data(self):
        pass

    def sensor_callback(self):
        pass

    def destroy(self):
        self.sensor.destroy()


class RovisServer(object):
    def init_server(self):
        self.host = enet.Host(enet.Address(self.ip, self.port), 10, 0, 0, 0)
        self.event = self.host.service(1000)
        threading.Thread(target=self.do_send).start()
        threading.Thread(target=self.do_service).start()

    def init_view(self):
        threading.Thread(target=self.do_view).start()