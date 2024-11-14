#pragma once
#include "resource.h"

class LoadingDialog : public CDialogEx
{
public:
    LoadingDialog(CWnd* pParent = nullptr); // Standard constructor

    CWnd* pParent;
    
    // Dialog Data
    enum { IDD = IDD_LOADING_DIALOG}; // IDD should be your dialog's resource ID


protected:
    virtual void DoDataExchange(CDataExchange* pDX); // DDX/DDV support

    CProgressCtrl m_progressBar; // Progress bar control

    afx_msg void OnClose();

public:
    void SetProgress(int progress); // Method to update the progress bar

    DECLARE_MESSAGE_MAP()
};

