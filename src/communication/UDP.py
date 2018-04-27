"""
    Nathaniel Goldfarb

"""
import socket
import struct
import array


class UDP(object):
    """open up an UDP connection. Used to talk the java hid2udp server."""
    def __init__(self, port):
        """

        :param port: UDP port to connect too
        """
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._server_address = ('localhost', self._port)


    def send_packet(self, board, id, packet):
        """

        :param board: ID of the board (int)
        :param id:  ID of the packet (int)
        :param packet: the packet to sent to the board
        :return: the message from the board
        """
        packet = list(packet)
        
        id_packet = [id]
        board_packet = [board]
        buf = struct.pack('%sf' % len(packet), *packet)
        buf_id = struct.pack('%si' % len(id_packet), *id_packet)
        buf_board = struct.pack('%si' % len(board_packet), *board_packet)
        sent = self._sock.sendto(buf_board+buf_id+buf, self._server_address)
        data, server = self._sock.recvfrom(self._port)

        return array.array('f',data )


