#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import una libreria standard di python per gestire il tempo
import time

import rospy
# Importo FLoat32 perchè è quello che ci serve, se servisse anche String farei per esempio import Float32, String
from std_msgs.msg import Float32

def calcolaR(a, b, t):
    """ La funzione chiesta
        parametri a, b, t Float
        return Float
    """
    return a*(t**2) + b*t

def talker():
    """ Ciao sono la descrizione di una funzione
    """
    # Salva il primo istante di avvio
    start_time = time.time()
    # Prepara il topic con nome risultato e che pubblica cose di tipo Float32
    pub = rospy.Publisher('risultato', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    # Fa ogni 1 secondo
    rate = rospy.Rate(1) # 1hz
    # Legge da terminale i valori che chiede all'utente
    a = input("Valore di a:")
    b = input("Valore di b:")
    while not rospy.is_shutdown():
        # Calcola da quanto tempo sta andando
        tempo = time.time() - start_time
        # Chiama la funzione sopra coi parametri a, b e tempo
        tempo_float = calcolaR(a, b, tempo)
        # Mostra i log a terminale
        rospy.loginfo(tempo_float)
        # Pubblica tempo_float nel canale
        pub.publish(tempo_float)
        # Aspetta a seconda del tempo dato da rate
        rate.sleep()

# Questa parte dello script parte solo se chiamata da terminale
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        # Se viene chiuso con ctrl+C parte la eccezione "ROSInterruptException" qua
        #  la prendo e semplicemente non faccio nulla così si chiude lo script
        pass
