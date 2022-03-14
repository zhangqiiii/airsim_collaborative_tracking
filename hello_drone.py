import multiprocessing


class S:
    c = 0
    f = 2

class A:
    def __init__(self, arr):
        self.s = None
        self.d = None

    def run(self):
        print("dd", self.s, self.d)
        self.stop()

    def stop(self):
        print("stop")


if __name__ == "__main__":
    llist = [None, None, None]
    for i in range(3):
        f = A(S())
        llist[i] = multiprocessing.Process(target=f.run)
        llist[i].start()
