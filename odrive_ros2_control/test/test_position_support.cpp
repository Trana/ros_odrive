#include "position_support.hpp"
#include "stop_admission.hpp"

#include <barrier>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace odrive_ros2_control;
void check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
int main() {
    try {
        PositionSupport support;
        support.configure(2);
        const std::vector<double> initial{.1, -.2}, native{.3, -.4}, changed{.5, -.6};
        support.begin(initial);
        check(support.producing_hold(), "activation owns measured hold");
        support.admit_native(5);
        support.retain();
        check(support.target(0) == .1, "an unsubmitted native target cannot replace support");
        support.completed(4, native);
        check(support.target(0) == .1, "old bootstrap completion cannot replace support");
        support.completed(5, native);
        check(support.target(0) == .3, "last partial native round may complete into retained support");
        support.admit_native(6);
        check(support.producing_hold(), "retired producer cannot resume");
        support.completed(6, changed);
        check(support.target(0) == .3, "non-native generation cannot replace support");
        support.stop();
        support.begin(changed);
        support.completed(5, native);
        check(support.target(0) == .5, "new activation retires old generation range");

        StopAdmission stop;
        auto token = stop.token();
        check(stop.admit(token), "initial explicit activation");
        {
            const auto request = stop.request();
            check(!stop.allowed(token), "Stop immediately fences admitted activation");
            check(!stop.admit(stop.token()), "activation cannot erase an in-progress Stop");
            {
                const auto other = stop.request();
                check(!stop.admit(stop.token()), "overlapping Stop cannot be erased");
            }
            check(!stop.admit(stop.token()), "first Stop remains in progress");
        }
        check(!stop.admit(token), "retired activation token cannot resume");
        check(stop.admit(stop.token()), "later activation may retire a completed Stop");
        for (int i = 0; i < 1000; ++i) {
            std::barrier rendezvous(2);
            const auto candidate = stop.token();
            std::thread stopper([&] {
                const auto request = stop.request();
                rendezvous.arrive_and_wait();
                rendezvous.arrive_and_wait();
            });
            rendezvous.arrive_and_wait();
            check(!stop.admit(candidate) && !stop.admit(stop.token()), "concurrent Stop must win");
            rendezvous.arrive_and_wait();
            stopper.join();
            check(stop.admit(stop.token()), "fresh activation after completed Stop");
        }
        for (int i = 0; i < 1000; ++i) {
            std::barrier rendezvous(2);
            const auto candidate = stop.token();
            std::thread stopper([&] {
                rendezvous.arrive_and_wait();
                const auto request = stop.request();
            });
            rendezvous.arrive_and_wait();
            (void)stop.admit(candidate);
            stopper.join();
            check(stop.requested() && !stop.allowed(candidate), "racing reset cannot erase the Stop latch");
            check(stop.admit(stop.token()), "new activation after racing Stop");
        }
        std::cout << "Position source ownership, coherent retention and Stop generation races passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
