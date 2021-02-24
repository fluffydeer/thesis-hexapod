package com.example.udp;

import android.os.Message;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;

public class UdpThread extends Thread{                  //po stlaceni tlacidla je vzdy na novo vytvorene nove vlakno, ktore sa stara o poslanie a nasledovne prijatie paketu
    String command;
    String ipAddress;
    int port;
    MainActivity.UdpHandler handler;
    DatagramSocket socket;
    InetAddress address;
    int timeout = 5000;                                 //cas po ktorom sa aplikacie prestane snazit nadviazat spojenie

    public UdpThread(String addr, int port, MainActivity.UdpHandler handler, String command) {
        super();
        this.command = command;
        this.ipAddress = addr;
        this.port = port;
        this.handler = handler;                         //povodny handler je potrebny na odosielanie sprav spat pre MainActivity - na aktualizovanie textovych poli
    }

    private void sendState(String state){               //retazec sa preposle do MainActivity.java, kde sa retazec zobrazi do textoveho pola
        handler.sendMessage(Message.obtain(handler, MainActivity.UdpHandler.UPDATE_STATE, state));
    }

    @Override
    public void run() {                                                     //posle spravu a potom cita spravu a ukonci sa vlakno, caka aj na tu spravu ci vie precitat aj prazdny buffer
        try {
            socket = new DatagramSocket();                                  //vytvorime socket
            address = InetAddress.getByName(ipAddress);                     //skontroluje ci je to ip adresa, ak nie vyhodi vynimku

            byte[] bufSend = command.getBytes();                            //pretypujeme prikaz, ktory chceme poslat ako paket
            DatagramPacket packet = new DatagramPacket(bufSend, bufSend.length, address, port);      //vytvori datagram packet prepojenu s  danou adresou a portom
            socket.send(packet);                                            //posleme datagram
            sendState("Packet sent.");

            byte[] bufReceive = new byte[256];                              //prazdny bajt na zachytenie poslanej spravy
            packet = new DatagramPacket(bufReceive, bufReceive.length);     //vytvori DatagramPacket na prijimanie paketov danej dlzky do daneho buffera
            socket.setSoTimeout(timeout);                                   //nastavime casovac
            socket.receive(packet);                                         //prijimeme paket
            String line = new String(packet.getData(), 0, packet.getLength());  //rozprarsujeme paket

            if(command.equals(line)){                                       //kontroluejeme ci sme dostali ten isty prikaz ako bol odoslany
                sendState("Successfully received response: " + line + "."); //TODO: funguje? skusit poslat iny paket z esp ci to prejde
            }else{
                sendState("Wrong packet. Could not receive response. Press the button again.");
            }
        } catch (SocketException e) {
            e.printStackTrace();
            sendState("SocketException");                                               //potrieda vynimky IOException, vyskytne sa pri problemoch so soketom (nadviazanie spojenia...)
        } catch(SocketTimeoutException e){
            e.printStackTrace();
            sendState("SocketTimeoutException. Could not receive the response. Press the button again.");
        } catch (UnknownHostException e) {
            e.printStackTrace();
            sendState("UnknownHostException. IPv4 address must be in format e.g. 0.1.2.3");
        } catch (IOException e) {                                                       //ak citame/zapisujeme data
            e.printStackTrace();
            sendState("IOException");
        } finally {                                                                     //tento blok sa uskutocni vzdy, aj ked je spojenie vytvorene aj ked nie je
            if(socket != null){
                socket.close();                                                         //tu vzdy sockety zatvorime, kvoli comu si musime vytvarat vzdy novu instanciu udpThread
                handler.sendEmptyMessage(MainActivity.UdpHandler.UPDATE_END);           //my tu posielame cisla reprezentujuce stav
            }
        }
    }
}
