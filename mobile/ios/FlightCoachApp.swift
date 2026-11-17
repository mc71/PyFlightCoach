import SwiftUI
import WebKit

final class FlightCoachEngine: ObservableObject {
    private var handle: OpaquePointer?
    @Published var htmlContent: String = ""
    @Published var summaryJson: String = "{}"
    @Published var selectedManoeuvre: Int = 1

    init() {
        self.handle = flightcoach_create()
        refresh()
    }

    deinit {
        if let h = handle {
            flightcoach_destroy(h)
        }
    }

    func selectManoeuvre(_ index: Int) {
        selectedManoeuvre = index
        guard let h = handle else { return }
        flightcoach_select_manoeuvre(h, Int32(index))
        refresh()
    }

    func loadBinData(_ data: Data) {
        guard let h = handle else { return }
        data.withUnsafeBytes { rawBuf in
            if let base = rawBuf.baseAddress?.assumingMemoryBound(to: UInt8.self) {
                _ = flightcoach_load_bin_bytes(h, base, data.count)
            }
        }
        refresh()
    }

    func refresh() {
        guard let h = handle else { return }
        if let cHtml = flightcoach_export_plotly_html(h) {
            htmlContent = String(cString: cHtml)
            flightcoach_free_string(cHtml)
        }
        if let cSum = flightcoach_get_summary_json(h) {
            summaryJson = String(cString: cSum)
            flightcoach_free_string(cSum)
        }
    }
}

struct PlotlyWebView: UIViewRepresentable {
    let html: String

    func makeUIView(context: Context) -> WKWebView {
        let webView = WKWebView()
        webView.isOpaque = false
        webView.backgroundColor = .black
        return webView
    }

    func updateUIView(_ uiView: WKWebView, context: Context) {
        uiView.loadHTMLString(html, baseURL: URL(string: "https://cdn.plot.ly"))
    }
}

@main
struct FlightCoachIOSApp: App {
    @StateObject private var engine = FlightCoachEngine()

    var body: some Scene {
        WindowGroup {
            NavigationSplitView {
                List {
                    Button("Full Flight") {
                        engine.selectManoeuvre(-1)
                    }
                    ForEach(1..<18, id: \.self) { idx in
                        Button("Manoeuvre #\(idx)") {
                            engine.selectManoeuvre(idx)
                        }
                    }
                }
                .navigationTitle("FlightCoach")
            } detail: {
                PlotlyWebView(html: engine.htmlContent)
                    .edgesIgnoringSafeArea(.all)
            }
        }
    }
}
