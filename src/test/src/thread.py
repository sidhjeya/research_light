import threading

def print_hi():
    print("hi")

def print_sidharth():
    print("sidharth")

if __name__ == "__main__":
    thread1 = threading.Thread(target=print_hi)
    thread2 = threading.Thread(target=print_sidharth)

    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()
