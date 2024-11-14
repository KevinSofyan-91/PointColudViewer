#include "pch.h"
#include "LoadingDialog.h"

#include "afxdialogex.h"
#include "LASViewerDlg.h"

LoadingDialog::LoadingDialog(CWnd* _pParent /*=nullptr*/)
    : CDialogEx(IDD_LOADING_DIALOG, pParent), pParent(_pParent)
{
}

void LoadingDialog::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_LOADING_PROGRESS, m_progressBar);
}

BEGIN_MESSAGE_MAP(LoadingDialog, CDialogEx)
    ON_WM_CLOSE()
END_MESSAGE_MAP()

void LoadingDialog::SetProgress(int progress)
{
    m_progressBar.SetPos(progress);
}

void LoadingDialog::OnClose()
{
    ShowWindow(SW_HIDE);
    m_progressBar.SetPos(0);

    if (pParent != nullptr)
    {
        pParent->PostMessage(WM_LOADING_CANCLED);
    }

    CDialogEx::OnClose();
}