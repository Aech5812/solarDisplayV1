#ifndef UART_GUARD_HPP
#define UART_GUARD_HPP

/**
 * @brief Simple RAII guard for UART thread-safe printing.
 * Expand this if you add an RTOS later (e.g., take a mutex in constructor, give in destructor).
 */
class UartGuard {
public:
    UartGuard() {}
    ~UartGuard() {}
};

#endif /* UART_GUARD_HPP */