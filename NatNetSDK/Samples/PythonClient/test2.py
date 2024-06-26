import natnetclient as natnet

client = natnet.NatClient(client_ip="192.168.0.181", server_ip="192.168.0.162", data_port=1511, comm_port=1510)
bot = client.rigid_bodies["Bot"]
print(bot.position)
