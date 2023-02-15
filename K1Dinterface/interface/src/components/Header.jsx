import { Container, Row, Col} from 'react-bootstrap';
import Logo from './../img/logo.png';

function Header() {
  return(
    <header>
      <Container fluid={true}>
        <Row>
          <Col md={4}>
            <img src={Logo} alt="Insper Dynamics" width={150} height={150}/>
          </Col>
          <Col md={{ span: 4, offset: 4 }}>
            <h1 className="mt-5">K1D Control Panel</h1>
          </Col>
        </Row>
      </Container>
    </header>
  );
    
}

export default Header;
