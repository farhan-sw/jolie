#include "FIR.h"
#include <Arduino.h> 

// Konstruktor
FIR::FIR(int filterOrder, float* coeffs) {
    order = filterOrder;
    
    // Alokasi memori untuk koefisien dan buffer
    coefficients = new float[order + 1];
    buffer = new float[order + 1];
    
    // Inisialisasi koefisien dan buffer
    for (int i = 0; i <= order; i++) {
        coefficients[i] = coeffs[i];
        buffer[i] = 0.0;
    }
}

// Destructor
FIR::~FIR() {
    delete[] coefficients;
    delete[] buffer;
}

// Fungsi untuk memproses input dan menghitung output
float FIR::process(float input) {
    // Geser buffer (dari belakang ke depan)
    for (int i = order; i > 0; i--) {
        buffer[i] = buffer[i - 1];
    }
    buffer[0] = input; // Masukkan nilai input baru ke buffer
    
    // Hitung output filter
    float output = 0.0;
    for (int i = 0; i <= order; i++) {
        output += coefficients[i] * buffer[i];
    }
    return output;
}
