#ifndef FIR_H
#define FIR_H

class FIR {
private:
    int order;                // Orde filter
    float* coefficients;      // Pointer untuk menyimpan array koefisien
    float* buffer;            // Buffer untuk menyimpan nilai input sebelumnya

public:
    // Konstruktor
    FIR(int filterOrder, float* coeffs);

    // Destructor
    ~FIR();

    // Fungsi untuk memproses input dan menghitung output
    float process(float input);
};

#endif
