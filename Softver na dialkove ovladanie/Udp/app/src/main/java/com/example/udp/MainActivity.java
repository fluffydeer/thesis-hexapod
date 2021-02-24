//TODO:
//komenty
//posielat rozne sekvencie? alebo to spravit, tak ze nech si to meni sam uzivatel v arduine?
//preskusat ci dokaze esp prijat prazdny packet a android prijat prazdny packet
//otestovat


package com.example.udp;

import android.os.Handler;
import android.os.Message;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;


public class MainActivity extends AppCompatActivity {

    EditText editTextAddress, editTextPort;     //polia na zadanie ip adresy a portu
    TextView textViewState;                     //text na spodku obrazovky opisujuci komunikaciu

    Button buttonForward;                       //tlacidla na ovladanie robota
    Button buttonBackward;
    Button buttonLeft;
    Button buttonRight;
    Button buttonTurnLeft;
    Button buttonTurnRight;
    Button buttonStandUp;
    Button buttonSitUp;
    Button buttonStop;

    UdpHandler udpHandler;                      //handler
    UdpThread udpThread;                        //vlakno, ktore sa vytvori po kliknuti na tlacidlo


    public class MyLovelyOnClickListener implements View.OnClickListener {      //implementacia mojho onClick listeneru
        String command;                                                         //premenna, do ktorej sa ulozi obsah paketu - prikaz pre hexapoda
        public MyLovelyOnClickListener(String command) {
            this.command = command;
        }

        @Override
        public void onClick(View v) {                                           //View v vyjadruje tlacidlo, na ktore bolo kliknute
            if(isInteger(editTextPort.getText().toString())){                   //skontrolujeme ci je port cislo, aby to nehadzalo vo vlakne vynimku
                sendCommand(command);                                           //posleme paket hexapodovi
            }
        }
    }

    boolean isInteger(String str){
        if(str.matches("^[1-9]\\d*$")){
            return true;
        }
        else {
            updateState("Port is not in correct format. It has to be an unsigned integer without leading zeros.");
            return false;
        }
    }

    void sendCommand(String command){                                           //funkcia na odoslanie paketu hexapodovi
        udpThread = new UdpThread(                                              //skonstruujeme novy objekt, do ktoreho posleme podtstane informacie
                editTextAddress.getText().toString(),
                Integer.parseInt(editTextPort.getText().toString()),
                udpHandler, command);
        udpThread.start();                                                      //objekt udpThread sam o sebe vytvori nove vlakno, ktore odosle a prijime datagram
    }


        @Override
    protected void onCreate(Bundle savedInstanceState) {                        //spusti sa pri vytvoreni triedy MainActivity
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        editTextAddress = (EditText) findViewById(R.id.address);                //odkazeme nase premenne na objekty v activity_main.xml
        editTextPort = (EditText) findViewById(R.id.port);
        textViewState = (TextView) findViewById(R.id.state);

        buttonForward = (Button) findViewById(R.id.buttonForward);              //odkazeme nase tlacidla na tlacidla v activity_main.xml
        buttonBackward = (Button) findViewById(R.id.buttonBackward);
        buttonLeft = (Button) findViewById(R.id.buttonLeft);
        buttonRight = (Button) findViewById(R.id.buttonRight);
        buttonTurnLeft = (Button) findViewById(R.id.buttonTurnLeft);
        buttonTurnRight = (Button) findViewById(R.id.buttonTurnRight);
        buttonStandUp = (Button) findViewById(R.id.buttonStandUp);
        buttonSitUp = (Button) findViewById(R.id.buttonSitUp);
        buttonStop = (Button) findViewById(R.id.buttonStop);

        buttonForward.setOnClickListener(new MyLovelyOnClickListener("FORWARD"));       //pridame listener na klikanie s paketom, ktory sa odosle po ich stlaceni
        buttonBackward.setOnClickListener(new MyLovelyOnClickListener("BACKWARD"));
        buttonLeft.setOnClickListener(new MyLovelyOnClickListener("LEFT"));
        buttonRight.setOnClickListener(new MyLovelyOnClickListener("RIGHT"));
        buttonTurnLeft.setOnClickListener(new MyLovelyOnClickListener("TURN_LEFT"));
        buttonTurnRight.setOnClickListener(new MyLovelyOnClickListener("TURN_RIGHT"));
        buttonStandUp.setOnClickListener(new MyLovelyOnClickListener("STAND_UP"));
        buttonSitUp.setOnClickListener(new MyLovelyOnClickListener("SIT_DOWN"));
        buttonStop.setOnClickListener(new MyLovelyOnClickListener("STOP"));

        udpHandler = new UdpHandler(this);                        //posleme mu instanciu sameho seba, je to vnorena trieda
    }

    private void updateState(String state) {                             //zobrazi podlany retazec v textovom poli dole na obrazovke
        textViewState.setText(state);
    }


    private void threadEnd() {                                          //ked udp vlakno skoncilo komunikaciu, tak vynulujeme premennu
        udpThread = null;
    }


    public static class UdpHandler extends Handler {                    //vytvorime si handler, ktory sa pouziva na aktualizovanie udajov v MainActivity z vlakna
        public static final int UPDATE_STATE = 0;                       //musia byt public, lebo su spistupnovane z UdpThread
        public static final int UPDATE_END = 1;                         //zmenit cislovanie tomuto
        private MainActivity parent;                                    //premenna sluziaca na spristupnenie funkcii v triede MainActivity

        public UdpHandler(MainActivity parent) {
            super();                                                    //zavola konstruktor triedy Handler od ktorej dedi
            this.parent = parent;
        }

        @Override
        public void handleMessage(Message msg) {                        //spravy posielane z UdpThread
            switch (msg.what) {                                         //poslane cisla repezentujuce horne premenne/stavy
                case UPDATE_STATE:
                    parent.updateState((String) msg.obj);               //ak posle UPDATE_STATE tak nastavi danu spravu textovemu polu pricom msg.obj je poslany retazec
                    break;
                case UPDATE_END:                                        //UPDATE_END vyjadruje ukoncenie vlakna
                    parent.threadEnd();
                    break;
                default:
                    super.handleMessage(msg);
            }
        }
    }
}
