import Sidebar from "@/components/Sidebar";
import RightSidebar from "@/components/RightSidebar";
import DemoModal from "@/components/DemoModal";
import PageTransition from "@/components/PageTransition";

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {

  return (
    <main className="flex flex-col">
      <DemoModal />
      <Sidebar />
      <RightSidebar/>
      <PageTransition>
        <div className="flex-1">{children}</div>
      </PageTransition>
    </main>
  );
}
