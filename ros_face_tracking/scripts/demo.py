import threading
import time


def do_work(id, stop):
    print("I am thread", id)
    while True:
        print("I am thread {} doing something".format(id))
        if stop():
            print("  Exiting loop.")
            break
    print("Thread {}, signing off".format(id))


def main():
    stop_threads = False
    workers = []
    for id in range(0, 3):
        tmp = threading.Thread(target=do_work, args=(id, lambda: stop_threads))
        workers.append(tmp)
        tmp.start()
    time.sleep(3)
    print('main: done sleeping; time to stop the threads.')
    stop_threads = True
    for worker in workers:
        worker.join()
    print('Finis.')


if __name__ == '__main__':
    main()