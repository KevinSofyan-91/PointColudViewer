#pragma once
#include "OpenGLRenderer.h"
#include "LoadingDialog.h"

#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>

// CLASViewerDlg dialog
class CLASViewerDlg : public CDialogEx
{
// Construction
public:
	CLASViewerDlg(CWnd* pParent = nullptr);	// standard constructor
	void LoadLasPointsInBackground(const CString& filePath);

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_LASVIEWER_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support

// Implementation
protected:
	HICON m_hIcon;
    CMenu m_Menu; // Declare a CMenu object

	CStatic m_OpenGLControl; // Custom control for OpenGL rendering

	LoadingDialog* m_LoadingDlg;

	// Thread Relative
	std::atomic<bool> m_bThreadRunning;  // Atomic flag to manage thread execution
	std::thread m_loadThread;            // The background thread
	void CLASViewerDlg::StopLoadingThread();

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg void OnDestroy();
    afx_msg void OnSize(UINT nType, int cx, int cy);
    afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
    afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
    afx_msg void OnMouseMove(UINT nFlags, CPoint point);
    afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg LRESULT OnLoadingCompleted(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnProgressUpdated(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnLoadingCancled(WPARAM wParam, LPARAM lParam);

    // Menu command handlers
    afx_msg void OnFileOpen();

	DECLARE_MESSAGE_MAP()

private:
	OpenGLRenderer* m_Render;
};
